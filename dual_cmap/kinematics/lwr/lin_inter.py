#!/usr/bin/python
# Copyright (c) 2009 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
# Author: Federico Ruiz-Ugalde <ruizf at in.tum.de>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
from numpy import array

#hack for avoiding stupid singularity on Kuka arm
def limit_kuka_to_positive(x):
  if x<0.5:
    return 0.5
  else:
    return x
def limit_kuka_to_negative(x):
  if x>-0.5:
    return -0.5
  else:
    return x
def fix_limit(x):
  if x[1]>x[2]:
    x[1]=x[2]
  return x

def a5a6_limit(a5,a6,limitsA5,limitsA6):
#    print "Angles received: ", a5, " ", a6
    a6lim=[0.0,0.0]
    for xtable,i in enumerate(limitsA6):
        if (a5==i[0]):
            #print i[1]
            a6lim=[i[1],i[2]]
        elif (i[0]>a5) and (limitsA6[xtable-1][0]<=a5):
            a6limL=lin_inter([i[0],i[1]],[limitsA6[xtable-1][0],limitsA6[xtable-1][1]],a5)
            a6limH=lin_inter([i[0],i[2]],[limitsA6[xtable-1][0],limitsA6[xtable-1][2]],a5)
            a6lim=[a6limL,a6limH]
    a5lim=[0.0,0.0]
    for xtable,i in enumerate(limitsA5):
        if (a6==i[0]):
            #print i[1]
            a5lim=[i[1],i[2]]
        elif (i[0]>a6) and (limitsA5[xtable-1][0]<=a6):
            a5limL=lin_inter([i[0],i[1]],[limitsA5[xtable-1][0],limitsA5[xtable-1][1]],a6)
            a5limH=lin_inter([i[0],i[2]],[limitsA5[xtable-1][0],limitsA5[xtable-1][2]],a6)
            a5lim=[a5limL,a5limH]
#    print "calculated limits: ", a5lim, " ", a6lim
    return a5lim,a6lim
            
def lin_inter(point1t,point2t,xt):
    '''Be sure to use doubles'''
    point1=point1t
    point2=point2t
    x=xt
    #print point1, point2
    if point1[0]>point2[0]:
        diff=array(point1)-array(point2)
        point=point2
    else:
        diff=array(point2)-array(point1)
        point=point1
    m=diff[1]/diff[0]
    y=m*x+(point[1]-m*point[0])
    return y
    

