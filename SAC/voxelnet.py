import torch
from torch import nn
from torch.nn import functional as F
from mmdet3d.registry import MODELS
from mmcv.cnn import ConvModule
from mmdet3d.models.data_preprocessors.voxelize import VoxelizationByGridShape

from mmdet3d.models.layers import SparseBasicBlock, make_sparse_convmodule
from mmdet3d.models.layers.spconv import IS_SPCONV2_AVAILABLE

import time

if IS_SPCONV2_AVAILABLE:
    from spconv.pytorch import SparseConvTensor, SparseSequential
else:
    from mmcv.ops import SparseConvTensor, SparseSequential

from point_utils.point_bert import TransformerEncoder

def tie_weights(src, trg):
    assert type(src) == type(trg)
    trg.weight = src.weight
    trg.bias = src.bias

class VoxelNet(nn.Module):

    def __init__(self):
        super().__init__()
        norm_cfg = dict(type='SyncBN', eps=1e-3, momentum=0.01)
        point_cloud_range = [-4.8, -4.8, -1.5, 4.8, 4.8, 1.5]
        voxel_size = [0.075, 0.075, 0.15]
        # point_cloud_range = [-9.6, -1.6, 0, 9.6, 0.448, 10]
        # voxel_size = [0.15, 0.016, 0.5]
        # point_cloud_range = [0, -9.6, -1.6, 10, 9.6, 0.448]
        # voxel_size = [0.5, 0.15, 0.016]
        self.voxel_layer = VoxelizationByGridShape(
            max_num_points=-1,  # max_points_per_voxel
            point_cloud_range=point_cloud_range,
            voxel_size=voxel_size)
        self.voxel_encoder = MODELS.build(dict(
            type='DynamicSimpleVFE',
            point_cloud_range=point_cloud_range,
            voxel_size=voxel_size))
        self.middle_encoder = SparseEncoder(
            in_channels=3,
            sparse_shape=[21, 128, 128],
            norm_cfg=norm_cfg,
            base_channels=16,
            output_channels=128,
            encoder_channels=((16, 16), (32, 32, 32), (64, 64, 64), (128, 128, 128)),
            encoder_paddings=((1, 1), (1, 1, 1), (1, 1, 1), (1, 1, 1)),
            block_type='conv_module',
        )
        # self.middle_encoder = SparseEncoder(
        #     in_channels=3,
        #     sparse_shape=[21, 128, 128],
        #     norm_cfg=norm_cfg,
        #     base_channels=16,
        #     output_channels=128,
        #     encoder_channels=((16, 16), (32, 32), (64, 64), (128, 128)),
        #     encoder_paddings=((1, 1), (1, 1), (1, 1), (1, 1)),
        #     block_type='conv_module',
        # )

        # self.conv = nn.Sequential(
        #     nn.AdaptiveMaxPool2d(1),
        #     nn.Flatten(),
        #     nn.Linear(128, 50),
        #     nn.Tanh()
        # )
    @torch.no_grad()
    def voxelize(self, points):
        coors = []
        # dynamic voxelization only provide a coors mapping
        for i, res in enumerate(points):
            res_coors = self.voxel_layer(res)
            res_coors = F.pad(res_coors, (1, 0), mode='constant', value=i)
            coors.append(res_coors)
        voxels = torch.cat(points, dim=0)
        coors = torch.cat(coors, dim=0)

        return voxels, coors

    def forward(self, x, detach=False):
        feats, coords = self.voxelize(x)
        voxel_features, coords = self.voxel_encoder(feats, coords)
        batch_size = coords[-1, 0] + 1
        encoder_features = self.middle_encoder(voxel_features, coords, batch_size) #[1, 128, 16, 16]
        # if detach:
        #     encoder_features.detach()
        return encoder_features





class SparseEncoder(nn.Module):
    r"""Sparse encoder for SECOND and Part-A2.

    Args:
        in_channels (int): The number of input channels.
        sparse_shape (list[int]): The sparse shape of input tensor.
        order (list[str], optional): Order of conv module.
            Defaults to ('conv', 'norm', 'act').
        norm_cfg (dict, optional): Config of normalization layer. Defaults to
            dict(type='BN1d', eps=1e-3, momentum=0.01).
        base_channels (int, optional): Out channels for conv_input layer.
            Defaults to 16.
        output_channels (int, optional): Out channels for conv_out layer.
            Defaults to 128.
        encoder_channels (tuple[tuple[int]], optional):
            Convolutional channels of each encode block.
            Defaults to ((16, ), (32, 32, 32), (64, 64, 64), (64, 64, 64)).
        encoder_paddings (tuple[tuple[int]], optional):
            Paddings of each encode block.
            Defaults to ((1, ), (1, 1, 1), (1, 1, 1), ((0, 1, 1), 1, 1)).
        block_type (str, optional): Type of the block to use.
            Defaults to 'conv_module'.
        return_middle_feats (bool): Whether output middle features.
            Default to False.
    """

    def __init__(self,
                 in_channels,
                 sparse_shape,
                 order=('conv', 'norm', 'act'),
                 norm_cfg=dict(type='BN1d', eps=1e-3, momentum=0.01),
                 base_channels=16,
                 output_channels=128,
                 encoder_channels=((16, ), (32, 32, 32), (64, 64, 64), (64, 64,
                                                                        64)),
                 encoder_paddings=((1, ), (1, 1, 1), (1, 1, 1), ((0, 1, 1), 1,
                                                                 1)),
                 block_type='conv_module',
                 return_middle_feats=False):
        super().__init__()
        assert block_type in ['conv_module', 'basicblock']
        self.sparse_shape = sparse_shape
        self.in_channels = in_channels
        self.order = order
        self.base_channels = base_channels
        self.output_channels = output_channels
        self.encoder_channels = encoder_channels
        self.encoder_paddings = encoder_paddings
        self.stage_num = len(self.encoder_channels)
        self.fp16_enabled = False
        self.return_middle_feats = return_middle_feats
        # Spconv init all weight on its own

        assert isinstance(order, tuple) and len(order) == 3
        assert set(order) == {'conv', 'norm', 'act'}

        if self.order[0] != 'conv':  # pre activate
            self.conv_input = make_sparse_convmodule(
                in_channels,
                self.base_channels,
                3,
                norm_cfg=norm_cfg,
                padding=1,
                indice_key='subm1',
                conv_type='SubMConv3d',
                order=('conv', ))
        else:  # post activate
            self.conv_input = make_sparse_convmodule(
                in_channels,
                self.base_channels,
                3,
                norm_cfg=norm_cfg,
                padding=1,
                indice_key='subm1',
                conv_type='SubMConv3d')

        encoder_out_channels = self.make_encoder_layers(
            make_sparse_convmodule,
            norm_cfg,
            self.base_channels,
            block_type=block_type)

        # self.conv_out = make_sparse_convmodule(
        #     encoder_out_channels,
        #     self.output_channels,
        #     kernel_size=(3, 1, 1),
        #     stride=(2, 1, 1),
        #     norm_cfg=norm_cfg,
        #     padding=0,
        #     indice_key='spconv_down2',
        #     conv_type='SparseConv3d')
        self.conv_out = ConvModule(encoder_out_channels * 3, self.output_channels, 1, norm_cfg=norm_cfg)

    def forward(self, voxel_features, coors, batch_size):
        """Forward of SparseEncoder.

        Args:
            voxel_features (torch.Tensor): Voxel features in shape (N, C).
            coors (torch.Tensor): Coordinates in shape (N, 4),
                the columns in the order of (batch_idx, z_idx, y_idx, x_idx).
            batch_size (int): Batch size.

        Returns:
            torch.Tensor | tuple[torch.Tensor, list]: Return spatial features
                include:

            - spatial_features (torch.Tensor): Spatial features are out from
                the last layer.
            - encode_features (List[SparseConvTensor], optional): Middle layer
                output features. When self.return_middle_feats is True, the
                module returns middle features.
        """
        coors = coors.int()
        input_sp_tensor = SparseConvTensor(voxel_features, coors,
                                           self.sparse_shape, batch_size)
        x = self.conv_input(input_sp_tensor)

        encode_features = []
        for encoder_layer in self.encoder_layers:
            x = encoder_layer(x)
            encode_features.append(x)

        # for detection head
        # [200, 176, 5] -> [200, 176, 2]
        # out = self.conv_out(encode_features[-1])
        spatial_features = encode_features[-1].dense()

        N, C, D, H, W = spatial_features.shape
        spatial_features = spatial_features.view(N, C * D, H, W)
        spatial_features = self.conv_out(spatial_features)

        if self.return_middle_feats:
            return spatial_features, encode_features
        else:
            return spatial_features

    def make_encoder_layers(self,
                            make_block,
                            norm_cfg,
                            in_channels,
                            block_type='conv_module',
                            conv_cfg=dict(type='SubMConv3d')):
        """make encoder layers using sparse convs.

        Args:
            make_block (method): A bounded function to build blocks.
            norm_cfg (dict[str]): Config of normalization layer.
            in_channels (int): The number of encoder input channels.
            block_type (str, optional): Type of the block to use.
                Defaults to 'conv_module'.
            conv_cfg (dict, optional): Config of conv layer. Defaults to
                dict(type='SubMConv3d').

        Returns:
            int: The number of encoder output channels.
        """
        assert block_type in ['conv_module', 'basicblock']
        self.encoder_layers = SparseSequential()

        for i, blocks in enumerate(self.encoder_channels):
            blocks_list = []
            for j, out_channels in enumerate(tuple(blocks)):
                padding = tuple(self.encoder_paddings[i])[j]
                # each stage started with a spconv layer
                # except the first stage
                if i != 0 and j == 0 and block_type == 'conv_module':
                    blocks_list.append(
                        make_block(
                            in_channels,
                            out_channels,
                            3,
                            norm_cfg=norm_cfg,
                            stride=2,
                            padding=padding,
                            indice_key=f'spconv{i + 1}',
                            conv_type='SparseConv3d'))
                elif block_type == 'basicblock':
                    if j == len(blocks) - 1 and i != len(
                            self.encoder_channels) - 1:
                        blocks_list.append(
                            make_block(
                                in_channels,
                                out_channels,
                                3,
                                norm_cfg=norm_cfg,
                                stride=2,
                                padding=padding,
                                indice_key=f'spconv{i + 1}',
                                conv_type='SparseConv3d'))
                    else:
                        blocks_list.append(
                            SparseBasicBlock(
                                out_channels,
                                out_channels,
                                norm_cfg=norm_cfg,
                                conv_cfg=conv_cfg))
                else:
                    blocks_list.append(
                        make_block(
                            in_channels,
                            out_channels,
                            3,
                            norm_cfg=norm_cfg,
                            padding=padding,
                            indice_key=f'subm{i + 1}',
                            conv_type='SubMConv3d'))
                in_channels = out_channels
            stage_name = f'encoder_layer{i + 1}'
            stage_layers = SparseSequential(*blocks_list)
            self.encoder_layers.add_module(stage_name, stage_layers)
        return out_channels


if __name__ == '__main__':

    points = [torch.randn(1024, 3, dtype=torch.float32).cuda(),
              torch.randn(1024, 3, dtype=torch.float32).cuda(),
              torch.randn(1024, 3, dtype=torch.float32).cuda()]
    # pcd = torch.randn(64, 3, 1024, 3, dtype=torch.float32).cuda()
    # pcd = pcd.view(-1, *pcd.shape[2:])
    # points = []
    # for t in pcd:
    #     points.append(t)
    model = VoxelNet()
    model.cuda().eval()
    with torch.no_grad():
        encoder_features = model(points)
        print(encoder_features.shape)





