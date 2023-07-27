lst = [5, 1, 4, 8, 2, 7]
new = []
length = len(lst)
i = 0
j = 0

while j <= length - 1:
    comp = lst[i] <= lst[j]
    if comp and j == length - 1:
        new.append(lst[i])
        lst[i] = []
        length -= 1
        i = 0
        j = 0
    elif not comp:
        i += 1
        j = 0
    else:
        j += 1

print(new)

