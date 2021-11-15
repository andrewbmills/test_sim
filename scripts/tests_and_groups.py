import os
import csv
import numpy as np
import matplotlib.pyplot as plt

def getDataSetSize(path, filename, first_test, last_test):
    for test_number in range(first_test, last_test):
        data = np.array(np.genfromtxt(path + "/" + filename + str(test_number) + "_map.csv", delimiter=",", skip_header=1))
        if (test_number == first_test):
            data_set_size = data.shape[0]
        else:
            data_set_size = np.min((data_set_size, data.shape[0]))
    return data_set_size

class Test:
    def __init__(self, filename, path):
        file_info = filename.split('_')
        self._id = int(file_info[2][-1])
        self._utility = file_info[3]
        self._utility_param = float(file_info[4])
        self._gain = file_info[5]
        data = np.array(np.genfromtxt(path + filename, delimiter=",", skip_header=2))
        self._t = data[:,0]/60.0
        self._voxel_size = np.genfromtxt(path + filename, skip_footer=len(self._t)+1)
        self._map_data = data[:,1]

class TestGroup:
    def __init__(self, test0):
        self.label=test0._utility + " " + str(test0._utility_param)
        self.tests = [test0]

def checkIfTestMatchesGroup(test, group):
    if (test._utility == group.tests[0]._utility and test._utility_param == group.tests[0]._utility_param and test._gain == group.tests[0]._gain):
        return True
    else:
        return False

def groupTests(tests):
    test_groups = []
    for test in tests:
        test_added = False
        for group in test_groups:
            if checkIfTestMatchesGroup(test, group):
                group.tests.append(test)
                test_added = True
                break
        if (test_added == False):
            test_groups.append(TestGroup(test))
    return test_groups

def averageGroup(group):
    min_data_length = 0
    for test in group.tests:
        if (min_data_length == 0):
            min_data_length = len(test._t)
        min_data_length = min(len(test._t), min_data_length)
    t = group.tests[0]._t[:min_data_length]
    map_datas = []
    for test in group.tests:
        if (len(map_datas) == 0):
            map_datas.append(test._map_data[:min_data_length])
        else:
            map_datas = np.vstack((map_datas, test._map_data[:min_data_length]))
    return t, np.mean(map_datas, axis=0)

def getMaxMapVoxelCount(tests):
    max_voxels = 0.0
    for test in tests:
        max_voxels = max(max_voxels, np.max(test._map_data))
    return max_voxels