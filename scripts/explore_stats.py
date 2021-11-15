import numpy as np

def get_exploration_cutoff_time(t, e_ratio, ratio_cutoff):
    # returns the time index when the exploration ratio (e_ratio) dips below ratio_cutoff.
    nonzero_indices = np.nonzero(e_ratio <= ratio_cutoff)
    if (len(nonzero_indices[0]) > 0):
        return t[nonzero_indices[0][0]]
    else:
        return t[-1]

def get_exploration_finish_time(t, e_ratio, finish_criteria, criteria_window):
    # calculates the time that exploration finishes according to a finish_criteria.
    # finish_criteria is the e_ratio range difference over some window that indicates exploration is finished.
    N = len(e_ratio)
    for i in range(0, N-criteria_window):
        exploration_finished = np.abs(e_ratio[i] - e_ratio[i+criteria_window]) <= finish_criteria
        if (exploration_finished):
            return t[i]
    return t[-1]

def get_ratio_at_time(t, e_ratio, t_target):
    # returns the e_ratio at the time immediately following t_target
    for i in range(0, len(t)):
        if (t[i] >= t_target):
            return e_ratio[i]
    return e_ratio[-1]

def get_cutoffs_finishes_and_ratios(t, e_ratio_mat, ratio_cutoff, finish_criteria, criteria_window, t_target):
    N, num_tests = e_ratio_mat.shape
    cutoff_times = np.array([])
    finish_times = np.array([])
    cutoff_ratios = np.array([])
    for i in range(0, num_tests):
        cutoff_times = np.append(cutoff_times, get_exploration_cutoff_time(t, e_ratio_mat[:,i], ratio_cutoff))
        finish_times = np.append(finish_times, get_exploration_finish_time(t, e_ratio_mat[:,i], finish_criteria, criteria_window))
        cutoff_ratios = np.append(cutoff_ratios, get_ratio_at_time(t, e_ratio_mat[:,i], t_target))
    return cutoff_times, finish_times, cutoff_ratios