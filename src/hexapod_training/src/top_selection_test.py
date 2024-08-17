import numpy

def replace_if_greater(numeric_arr, side_list, numeric_val, side_val):
    if numpy.size(numeric_arr, 0) < 10:
        numeric_arr = numpy.append(numeric_arr, numeric_val)
        side_list.append(side_val)
        return numeric_arr

    # Find the index of the tuple to replace
    replace_index = numpy.where(numeric_arr < numeric_val)[0]

    # Check if there are any valid indices
    if replace_index.size > 0:
        # Find the index of the tuple with the minimum first value that's less than the new tuple's first value
        min_index = replace_index[numpy.argmin(numeric_arr[replace_index])]
        
        # Replace the tuple at that index
        numeric_arr[min_index] = numeric_val
        side_list[min_index] = side_val
    return numeric_arr

arr = numpy.array([])
side_list = []
for i in range(10):
    arr = replace_if_greater(arr, side_list, i+1, 1)
print(arr)
print(side_list)
arr = replace_if_greater(arr, side_list, 13, 55)
# arr = replace_if_greater(arr, side_list, 5, 1)
print(arr)
print(side_list)