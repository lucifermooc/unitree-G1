# coding=utf8
import os
import sqlite3
from ament_index_python.packages import get_package_prefix



class SQLite(object):
    conn = None
    cursor = None

    def __init__(self, db_file_absolute_path=None):
        pass

    def __new__(cls, *args, **kwargs):
        print("singleton")
        # 单例模式下如何存在实例，就返回实例
        if not hasattr(cls, 'instance'):
            print("using new obj")
            cls.instance = super().__new__(cls)
            if "db_file_absolute_path" in kwargs:
                cls.instance.db_file_path = kwargs.get("db_file_absolute_path")
            elif len(args):
                cls.instance.db_file_path = args[0]
            else:
                raise Exception("db_file_absolute_path is required")
            db_exists = os.path.exists(cls.instance.db_file_path)
            cls.instance.conn = sqlite3.connect(cls.instance.db_file_path)
            cls.instance.conn.row_factory = cls.dict_factory
            cls.instance.cursor = cls.instance.conn.cursor()
            if not db_exists:
                sql_file_dir = os.path.join(get_package_prefix('aid_robot_py'))
                init_db_sql_file_path = sql_file_dir + '/db/db.sql'
                print(f"db init sql file is {init_db_sql_file_path}")
                with open(init_db_sql_file_path, 'r', encoding='utf8') as f:
                    cls.instance.cursor.executescript(f.read())
        else:
            print("using current obj")
        return cls.instance

    @classmethod
    def dict_factory(cls, cursor, row):
        d = {}
        for idx, col in enumerate(cursor.description):
            d[col[0]] = row[idx]
        return d

    def disconnect(self):
        self.cursor.close()
        self.conn.close()

    def execSQL(self, sql, value=None):
        # 增、删、查、改
        if isinstance(value, list) and isinstance(value[0], (list, tuple)):
            for v in value:
                self.cursor.execute(sql, v)
            else:
                self.conn.commit()
                result = self.cursor.fetchall()
        else:
            # 执行单条语句：字符串、整型、数组
            if value:
                self.cursor.execute(sql, value)
            else:
                self.cursor.execute(sql)
            self.conn.commit()
            result = self.cursor.fetchall()

        return result
