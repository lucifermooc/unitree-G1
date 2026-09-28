"""BGE-M3：文本 → 1024 维 dense 向量。模型只加载一次。"""
import threading


class Embedder:
    def __init__(self, model_name="BAAI/bge-m3", use_fp16=False):
        self.model_name = model_name
        self.use_fp16 = use_fp16
        self._model = None
        self._lock = threading.Lock()

    def load(self):
        with self._lock:
            if self._model is None:
                from FlagEmbedding import BGEM3FlagModel
                self._model = BGEM3FlagModel(self.model_name, use_fp16=self.use_fp16)
        return self._model

    def encode(self, texts):
        model = self.load()
        with self._lock:  # 模型推理不是线程安全的
            out = model.encode(texts, batch_size=16, max_length=512, return_dense=True,
                               return_sparse=False, return_colbert_vecs=False)
        return [v.tolist() for v in out["dense_vecs"]]
