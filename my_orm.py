"""
This module provides the interface to easy interaction with DB for simple commands. Contains create_table, write_the_value, read_the_value functions.
Model for all DB:
1) ID : int
2) model_name : text
3) shell_angles : json (text)
4) langeron_angles : json (text)
5) langeron_wall_angles : json(text)
6) value_vertical : real
7) value_horizontal : real
8) value_neutral : real
9) value_spectrum_tang : json (text)
10) value_spectrum_attack : json (text)
11) value_spectrum_roll : json (text)
12) antiflatter_x : real
13) antiflatter_y : real
14) antiflatter_diam : real
15) antiflatter_lenght : real
15) wall_lenght : real
16) wall_angle : real
"""

import json, sqlite3
from sqlalchemy import Column, Integer, String, Table
from sqlalchemy.ext.declarative import declarative_base


Base = declarative_base()


class ShellModel(Base):
    """Default model of wing without longerone"""
    def __init__(self, **kwargs):
        self.ID = kwargs['ID']
        self.model_name = kwargs['model_name']
        self.shell_angles = kwargs['shell_angles']
        self.value_vertical = kwargs['value_vertical']
        self.value_horizontal = kwargs['value_horizontal']
        self.value_neutral = kwargs['value_neutral']
        self.value_spectrum_tang = kwargs['value_spectrum_tang']
        self.value_spectrum_attack = kwargs['value_spectrum_attack']
        self.value_spectrum_roll = kwargs['value_spectrum_roll']
        self.antiflatter_x = kwargs['antiflatter_x']
        self.antiflatter_y = kwargs['antiflatter_y']
        self.antiflatter_diam = kwargs['antiflatter_diam']
        self.antiflatter_lenght = kwargs['antiflatter_lenght']
    
    __tablename__ = 'Shell'
    
    def pull_all_values_in_db(self):
        pass

    def pull_current_value_in_db(self, value):
        pass


class LangeronModel(ShellModel):
    """Default model of wing with longerone"""
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.langeron_angles = kwargs['langeron_angles']
        self.langeron_wall_angles = kwargs['langeron_wall_angles']
        self.antiflatter_lenght = kwargs['wall_lenght']
        self.antiflatter_lenght = kwargs['wall_angle']