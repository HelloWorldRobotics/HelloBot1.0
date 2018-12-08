#!/usr/bin/env python  
# -*- coding: utf-8 -*-

#常量定义类
class _const:
  class ConstError(TypeError):pass
  def __setattr__(self,name,value):
    #if self.__dict__.has_key(name): 
    if name in self.__dict__:
      raise self.ConstError("Can't rebind const(%s)"%name)
    self.__dict__[name] = value
import sys
sys.modules[__name__] = _const()
