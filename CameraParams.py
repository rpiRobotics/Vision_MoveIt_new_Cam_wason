# -*- coding: utf-8 -*-
"""
Created on Wed Feb 28 10:56:11 2018

@author: d0v0b
"""
import numpy as np

class CameraParams:
    def __init__(self,M00,M02,M11,M12,M22,C00,C01,C02,C03,C04):
        self.camMatrix, self.distCoeff = self.CameraParams(M00,M02,M11,M12,M22,C00,C01,C02,C03,C04)
    
        
    def CameraParams(self, M00,M02,M11,M12,M22,C00,C01,C02,C03,C04):
        camMatrix = np.zeros((3, 3),dtype=np.float64)
        camMatrix[0][0] = M00
        camMatrix[0][2] = M02
        camMatrix[1][1] = M11
        camMatrix[1][2] = M12
        camMatrix[2][2] = M22

        distCoeff = np.zeros((1, 5), dtype=np.float64)
        distCoeff[0][0] = C00
        distCoeff[0][1] = C01
        distCoeff[0][2] = C02
        distCoeff[0][3] = C03
        distCoeff[0][4] = C04

        return camMatrix,distCoeff
