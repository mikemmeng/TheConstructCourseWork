# Create a list of lists
matrix = [
    [1, 2, 3],
    [4, 5, 6],
    [7, 8, 9],
    [10, 11, 12],
    [23, 45, 67]
]

# Pop the first element from each sublist
for sublist in matrix:
    sublist.pop(0)

# Access elements in the modified list of lists
first_element = matrix[0][0]  # 2
second_row_third_element = matrix[1][2]  # 6
third_row_second_element = matrix[2][1]  # 9

# Print accessed elements
print("First element:", first_element)
print("Second row, third element:", second_row_third_element)
print("Third row, second element:", third_row_second_element)