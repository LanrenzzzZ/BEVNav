import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.nn import LayerNorm

class PointNet(nn.Sequential):

    def __init__(self):
        super(PointNet, self).__init__(
            nn.Conv1d(3, 64, 1, bias=False),
            nn.BatchNorm1d(64, eps=1e-3, momentum=0.01),
            nn.ReLU(True),
            nn.Conv1d(64, 128, 1, bias=False),
            nn.BatchNorm1d(128, eps=1e-3, momentum=0.01),
            nn.ReLU(True),
            nn.Conv1d(128, 256, 1, bias=False),
            nn.BatchNorm1d(256, eps=1e-3, momentum=0.01),
            nn.ReLU(True),
            nn.AdaptiveMaxPool1d(1),
            nn.Flatten(),
            nn.Linear(256, 50),
            nn.ReLU(True)
        )

    def forward(self, points):
        """
        points: tensor [B, N, 3],
        return: [B, 128]
        """
        points = points.transpose(1, 2).contiguous()
        return super(PointNet, self).forward(points)

class MultiheadSelfAttention(nn.Module):

    def __init__(self, embed_dim, num_heads=1, dropout=0):
        super().__init__()
        self.embed_dim = embed_dim
        self.num_heads = num_heads

        self.head_dim = embed_dim // num_heads
        assert embed_dim % num_heads == 0
        self.scaling = self.head_dim ** -0.5
        self.in_proj = nn.Linear(embed_dim, 3 * embed_dim)
        self.dropout = dropout

    def forward(self, x):
        tgt_len, bsz, embed_dim = x.size()
        x = self.in_proj(x)
        q, k, v = x.chunk(3, dim=-1)
        q = q * self.scaling

        q = q.contiguous().view(tgt_len, bsz * self.num_heads, self.head_dim).transpose(0, 1)
        k = k.contiguous().view(-1, bsz * self.num_heads, self.head_dim).transpose(0, 1)
        v = v.contiguous().view(-1, bsz * self.num_heads, self.head_dim).transpose(0, 1)

        # attn weight [bsz * num_heads, tgt_len, src_len]
        attn_weights = torch.bmm(q, k.transpose(1, 2))
        attn_weights = attn_weights - torch.max(attn_weights, dim=-1, keepdim=True)[0]
        attn_weights = F.softmax(attn_weights, dim=-1)
        attn_weights = F.dropout(attn_weights, p=self.dropout, training=self.training)
        attn = torch.bmm(attn_weights, v)
        attn = attn.transpose(0, 1).contiguous().view(tgt_len, bsz, embed_dim)
        return attn

class MultiheadCrossAttention(nn.Module):

    def __init__(self, embed_dim, num_heads=1, dropout=0):
        super().__init__()
        self.embed_dim = embed_dim
        self.num_heads = num_heads

        self.head_dim = embed_dim // num_heads
        assert embed_dim % num_heads == 0
        self.scaling = self.head_dim ** -0.5
        self.q = nn.Linear(embed_dim, embed_dim)
        self.k = nn.Linear(embed_dim, embed_dim)
        self.v = nn.Linear(embed_dim, embed_dim)
        self.dropout = dropout

    def forward(self, x_t, x_t_1):
        tgt_len, bsz, embed_dim = x_t.size()
        q = self.q(x_t_1)
        k = self.k(x_t)
        v = self.v(x_t)
        q = q * self.scaling

        q = q.contiguous().view(tgt_len, bsz * self.num_heads, self.head_dim).transpose(0, 1)
        k = k.contiguous().view(-1, bsz * self.num_heads, self.head_dim).transpose(0, 1)
        v = v.contiguous().view(-1, bsz * self.num_heads, self.head_dim).transpose(0, 1)

        # attn weight [bsz * num_heads, tgt_len, src_len]
        attn_weights = torch.bmm(q, k.transpose(1, 2))
        attn_weights = attn_weights - torch.max(attn_weights, dim=-1, keepdim=True)[0]
        attn_weights = F.softmax(attn_weights, dim=-1)
        attn_weights = F.dropout(attn_weights, p=self.dropout, training=self.training)
        attn = torch.bmm(attn_weights, v)
        attn = attn.transpose(0, 1).contiguous().view(tgt_len, bsz, embed_dim)
        return attn

class EncoderLayer(nn.Module):

    def __init__(self, embed_dim, normalize_before=False,
                 relu_dropout=0.,
                 attention_dropout=0.,
                 dropout=0.,
                 ):
        super().__init__()
        self.self_attn = MultiheadSelfAttention(embed_dim, dropout=attention_dropout)
        self.attn_layer_norm = LayerNorm(embed_dim)
        self.fc1 = nn.Linear(embed_dim, 2 * embed_dim)
        self.fc2 = nn.Linear(2 * embed_dim, embed_dim)
        self.final_layer_norm = LayerNorm(embed_dim)
        self.normalize_before = normalize_before
        self.dropout = dropout
        self.relu_dropout = relu_dropout

    def forward(self, x):
        residual = x
        x = self.maybe_layer_norm(self.attn_layer_norm, x, before=True)
        x = self.self_attn(x)
        x = F.dropout(x, p=self.dropout, training=self.training)
        x = residual + x
        x = self.maybe_layer_norm(self.attn_layer_norm, x, after=True)

        residual = x
        x = self.maybe_layer_norm(self.final_layer_norm, x, before=True)
        x = F.relu(self.fc1(x))
        x = F.dropout(x, p=self.relu_dropout, training=self.training)
        x = self.fc2(x)
        x = F.dropout(x, p=self.dropout, training=self.training)
        x = residual + x
        x = self.maybe_layer_norm(self.final_layer_norm, x, after=True)

        return x

    def maybe_layer_norm(self, layer_norm, x, before=False, after=False):
        assert before ^ after
        if after ^ self.normalize_before:
            return layer_norm(x)
        else:
            return x


class DecoderLayer(nn.Module):

    def __init__(self, embed_dim, normalize_before=False,
                 relu_dropout=0.,
                 attention_dropout=0.,
                 dropout=0.,
                 ):
        super().__init__()
        self.self_attn = MultiheadCrossAttention(embed_dim, dropout=attention_dropout)
        self.attn_layer_norm = LayerNorm(embed_dim)
        self.fc1 = nn.Linear(embed_dim, 2 * embed_dim)
        self.fc2 = nn.Linear(2 * embed_dim, embed_dim)
        self.final_layer_norm = LayerNorm(embed_dim)
        self.normalize_before = normalize_before
        self.dropout = dropout
        self.relu_dropout = relu_dropout

    def forward(self, x_t, x_t_1):
        residual = x_t
        x_t = self.maybe_layer_norm(self.attn_layer_norm, x_t, before=True)
        x_t_1 = self.maybe_layer_norm(self.attn_layer_norm, x_t_1, before=True)
        x = self.self_attn(x_t, x_t_1)
        x = F.dropout(x, p=self.dropout, training=self.training)
        x = residual + x
        x = self.maybe_layer_norm(self.attn_layer_norm, x, after=True)

        residual = x
        x = self.maybe_layer_norm(self.final_layer_norm, x, before=True)
        x = F.relu(self.fc1(x))
        x = F.dropout(x, p=self.relu_dropout, training=self.training)
        x = self.fc2(x)
        x = F.dropout(x, p=self.dropout, training=self.training)
        x = residual + x
        x = self.maybe_layer_norm(self.final_layer_norm, x, after=True)

        return x

    def maybe_layer_norm(self, layer_norm, x, before=False, after=False):
        assert before ^ after
        if after ^ self.normalize_before:
            return layer_norm(x)
        else:
            return x

class Pointnet_transforemr(nn.Module):
    def __init__(self,
                z_dim,
                normalize_before = False,
                dropout=0.0,
                attention_dropout=0.0,
                relu_dropout=0.0,
                num_attn_layer=1):
        super().__init__()
        # self.pointnet = PointNet()
        self.encoder_layers = nn.ModuleList([EncoderLayer(z_dim, normalize_before=normalize_before,
                                                    dropout=dropout, attention_dropout=attention_dropout,
                                                    relu_dropout=relu_dropout)
                                          for _ in range(num_attn_layer)])
        self.decoder_layer = nn.ModuleList([DecoderLayer(z_dim, normalize_before=normalize_before,
                                                    dropout=dropout, attention_dropout=attention_dropout,
                                                    relu_dropout=relu_dropout)
                                          for _ in range(num_attn_layer)])
    def forward(self, x_t, x_t_1):
        for i in range(len(self.encoder_layers)):
            x_t = self.encoder_layers[i](x_t)
            x_t_1 = self.encoder_layers[i](x_t_1)
        for i in range(len(self.decoder_layer)):
            x = self.decoder_layer[i](x_t, x_t_1)
        return x

if __name__ == "__main__":
    points = torch.randn([1, 512, 3])
    x_t = torch.randn([1, 512, 128])
    x_t_1 = torch.randn([1, 512, 128])
    encoder = Pointnet_transforemr(z_dim=128)
    print(encoder(x_t, x_t_1).shape)
    model = PointNet()
    print(model(points).shape)

