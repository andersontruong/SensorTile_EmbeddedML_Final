import random

count = int(input('How many weights? '))

with open('weights.txt', 'w+') as f:
    f.write(f'float weights[{count}] = {{')
    weights = [round(random.random(), 4) for i in range(count)]
    weights_string = [f'{weight:<08},' if (i+1)%5 != 0 else f'{weight:<08},\n                     ' for i, weight in enumerate(weights)]

    csv_string = ' '.join(weights_string)
    csv_string = csv_string.rstrip(',')
    csv_string += '};'

    f.write(csv_string)
