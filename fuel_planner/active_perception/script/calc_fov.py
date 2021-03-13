from math import *
import numpy as np

# cam_fx:     387.229
# cam_fy:     387.229
# cam_cx:     320.000
# cam_cy:     243.449
# # cam_cx:     321.046

# proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
# proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;

left = atan2(321.046, 387.229)
right = atan2(640-321.046, 387.229)
up = atan2(243.449, 387.229)
print left
print right
print up
print left * 57.3
print up * 57.3
# print 0.5 * (left - right)
# up = atan2(236.743, 257.296)

print exp(0.5)**2
print exp(0.2)**5

a1 = np.array([29.08, 31.51, 32.22, 28.65, 28.95, 28.7, 30.84, 34.29])
a2 = np.array([31.50, 27.20, 30.90, 32.74, 29.65, 30.6, 29.93])

print a1.mean()
print a1.var()
print a1.max()

print a2.mean()
print a2.var()
print a2.max()