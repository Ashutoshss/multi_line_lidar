count = 0
direction = 1

while True:
    print(count)
    if(count==180):
        direction = -1
    if(count==0):
        direction = 1

    count +=direction