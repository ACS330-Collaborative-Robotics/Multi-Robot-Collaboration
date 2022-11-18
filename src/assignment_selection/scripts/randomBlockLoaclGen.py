import random

blocks = 10
robots = 2
decimalplace = 2
output = []

for i in range(blocks+robots):
    output.append([random.randint(0,999)/10**decimalplace,random.randint(0,999)/10**decimalplace])

print(output)