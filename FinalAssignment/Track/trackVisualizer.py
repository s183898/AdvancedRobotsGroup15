import matplotlib.pyplot as plt

coordinates = {}

paths = []

with open('track.txt','r') as f:
    lines = f.readlines()
    for line in lines:

        xi = line.find("x=")
        yi = line.find("y=")
        i = line.find("pno=")

        pi1 = line.find("pno1=")
        pi2 = line.find("pno2=")

        if (pi1 != -1 & pi2 != -1):
            p1 = int(line[pi1+5:pi2-1])
            p2 = int(line[pi2+5:-3])
            paths.append([p1,p2])

        if (xi != -1 & yi != -1 & i != -1):
            x = float(line[xi+2:yi-1])
            y = float(line[yi+2:-3])

            coordinates[int(line[i+4:xi-1])] = (x,y)

for path in paths:
    print(path)
    plt.plot([coordinates[path[0]][0],coordinates[path[1]][0]],[coordinates[path[0]][1],coordinates[path[1]][1]], c = "red")

plt.scatter([x[0] for x in coordinates.values()], [y[1] for y in coordinates.values()])
plt.axis('equal')
plt.show()


