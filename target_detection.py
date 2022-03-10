"""
    Imports
"""
import numpy as np
from scipy.interpolate import interp1d

"""
    Functions
"""
def cfar(X_k, num_guard_cells, num_ref_cells, bias, cfar_method='average'):
    N = X_k.size
    cfar_values = np.ma.masked_all(X_k.shape)
    for center_index in range(num_guard_cells + num_ref_cells, N - (num_guard_cells + num_ref_cells)):
        min_index = center_index - (num_guard_cells + num_ref_cells)
        min_guard = center_index - num_guard_cells 
        max_index = center_index + (num_guard_cells + num_ref_cells) + 1
        max_guard = center_index + num_guard_cells + 1

        lower_nearby = X_k[min_index:min_guard]
        upper_nearby = X_k[max_guard:max_index]

        lower_mean = np.mean(lower_nearby)
        upper_mean = np.mean(upper_nearby)

        if (cfar_method == 'average'):
            mean = np.mean(np.concatenate((lower_nearby, upper_nearby)))
        elif (cfar_method == 'greatest'):
            mean = max(lower_mean, upper_mean)
        elif (cfar_method == 'smallest'):
            mean = min(lower_mean, upper_mean)
        else:
            mean = 0

        output = mean * bias
        cfar_values[center_index] = output

    cfar_values[np.where(cfar_values == np.ma.masked)] = np.min(cfar_values)

    targets_only = np.ma.masked_array(np.copy(X_k))
    targets_only[np.where(abs(X_k) < abs(cfar_values))] = np.ma.masked

    return cfar_values, targets_only
