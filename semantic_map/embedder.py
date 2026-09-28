"""BGE-M3 封装：文本 → 1024 维 dense 向量。

建库和查询必须用同一个模型、同样的参数，否则向量不在同一个空间里，搜不准。
"""
import config

_model = None


def load_model():
    """加载 BGE-M3（只加载一次）。第一次运行会自动下载约 2.3GB 模型文件。"""
    global _model
    if _model is None:
        from FlagEmbedding import BGEM3FlagModel

        print(f"正在加载模型 {config.MODEL_NAME} ...")
        _model = BGEM3FlagModel(config.MODEL_NAME, use_fp16=config.USE_FP16)
    return _model


def encode(texts):
    """批量把文本转成向量，返回 list[list[float]]，每条 1024 个数。"""
    model = load_model()
    output = model.encode(
        texts,
        batch_size=16,
        max_length=512,
        return_dense=True,
        return_sparse=False,
        return_colbert_vecs=False,
    )
    return [vec.tolist() for vec in output["dense_vecs"]]
