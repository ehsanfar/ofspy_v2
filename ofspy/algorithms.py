import numpy as np
import gym



def list2dict(l):
    dic = {}
    for e in l:
        dic[e] = dic[e]+1 if e in dic else 1

    return dic