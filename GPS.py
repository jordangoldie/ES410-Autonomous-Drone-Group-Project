import math

lat = math.radians(-35.36275)
long = math.radians(149.1642)
R = 6378100

x = R * math.cos(lat) * math.cos(long)
y = R * math.cos(lat) * math.sin(long)
z = R * math.sin(lat)

print (f'x: {x}, y: {y}, z: {z}')

