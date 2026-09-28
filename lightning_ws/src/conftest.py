# 跑测试时不在源码树里留 __pycache__/*.pyc：之后导入的测试模块、launch 文件都不再写字节码。
# 本文件自己在这行生效前已被 pytest 编译并写了缓存，顺手删掉（src/__pycache__ 里只有它）。
import pathlib
import shutil
import sys

sys.dont_write_bytecode = True
shutil.rmtree(pathlib.Path(__file__).with_name('__pycache__'), ignore_errors=True)
