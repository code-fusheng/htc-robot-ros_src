#!/usr/bin/python
# -*-coding:utf-8-*-
import tf


class Path:
    def __init__(self):
        self.a = None
        self.b = None
        self.c = []


q = [0, 0, 0, 1]
print(tf.transformations.euler_from_quaternion(q))
