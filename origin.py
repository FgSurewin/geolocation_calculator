import csv
import utm
import numpy as np
import json
import requests
import time
import warnings

warnings.filterwarnings("ignore")  # setting ignore as a parameter


def csvToArray(fileName):
    data = []
    with open(fileName, "r") as csvfile:
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            data.append(row)
    return data


def writeCSV(data, fileName):
    with open(fileName, "w", newline="") as csvfile:
        writer = csv.writer(csvfile, delimiter=",")
        for ar in data:
            writer.writerow(ar)


def findIndex(x, y):
    return int((y - 4488500) / 100) * 271 + int((x - 581000) / 100)


def parseMultistring(polygon):
    polygon = polygon[16:-3]
    stringcoords = polygon.split(", ")
    result = []
    for ar in stringcoords:
        curr = ar.split(" ")
        if curr[0][0] == "(":
            curr[0] = curr[0][1:]
        if curr[1][-1] == ")":
            curr[1] = curr[1][:-1]
        result.append(utm.from_latlon(float(curr[1]), float(curr[0]))[0:2])
    return result


def magnitude(vector):
    return np.sqrt(np.dot(np.array(vector), np.array(vector)))


def norm(vector):
    return np.array(vector) / magnitude(np.array(vector))


def findDoorCoords(coords, angle, possible):
    minDist = 999999999
    result = []
    doorUTM = utm.from_latlon(float(coords[0]), float(coords[1]))[0:2]
    for ar in possible:
        if len(ar) == 4:
            continue
        storeEdges = parseMultistring(ar[4])
        for i in range(len(storeEdges)):
            curr = storeEdges[i]
            adj = storeEdges[(i + 1) % len(storeEdges)]
            intersect = lineRayIntersectionPoint(
                doorUTM,
                [np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle))],
                curr,
                adj,
            )
            if len(intersect) == 0:
                continue
            currDist = np.sqrt(
                (intersect[0] - doorUTM[0]) ** 2 + (intersect[1] - doorUTM[1]) ** 2
            )
            if currDist < minDist:
                minDist = currDist
                result = intersect
    return result


# https://stackoverflow.com/questions/14307158/how-do-you-check-for-intersection-between-a-line-segment-and-a-line-ray-emanatin
def lineRayIntersectionPoint(rayOrigin, rayDirection, point1, point2):
    # Convert to numpy arrays
    rayOrigin = np.array(rayOrigin, dtype=np.float)
    rayDirection = np.array(norm(rayDirection), dtype=np.float)
    point1 = np.array(point1, dtype=np.float)
    point2 = np.array(point2, dtype=np.float)

    # Ray-Line Segment Intersection Test in 2D
    # http://bit.ly/1CoxdrG
    v1 = rayOrigin - point1
    v2 = point2 - point1
    v3 = np.array([-rayDirection[1], rayDirection[0]])
    t1 = np.cross(v2, v1) / np.dot(v2, v3)
    t2 = np.dot(v1, v3) / np.dot(v2, v3)
    if t1 >= 0.0 and t2 >= 0.0 and t2 <= 1.0:
        return rayOrigin + t1 * rayDirection
    return []


# https://stackoverflow.com/questions/64775547/extract-depthmap-from-google-street-view
def getHeading(lat, lon):
    url = "https://maps.googleapis.com/maps/api/js/GeoPhotoService.SingleImageSearch?pb=!1m5!1sapiv3!5sUS!11m2!1m1!1b0!2m4!1m2!3d{0:}!4d{1:}!2d50!3m10!2m2!1sen!2sGB!9m1!1e2!11m4!1m3!1e2!2b1!3e2!4m10!1e1!1e2!1e3!1e4!1e8!1e6!5m1!1e2!6m1!1e2&callback=_xdc_._v2mub5"
    url = url.format(lat, lon)
    resp = requests.get(url, proxies=None)
    line = resp.text.replace("/**/_xdc_._v2mub5 && _xdc_._v2mub5( ", "")[:-2]
    jdata = json.loads(line)
    return jdata[1][5][0][1][2][0]


def main(camCoord, heading):
    grid = csvToArray("./origin_data/gridData.csv")
    PLUTOArray = csvToArray("./origin_data/PLUTOFootprint.csv")

    # Local Files
    curr = utm.from_latlon(camCoord[0], camCoord[1])

    surroundingI = []
    for i in range(-2, 3):
        for j in range(-2, 3):
            currIndex = findIndex(curr[0] + i * 100, curr[1] + j * 100)
            if currIndex not in surroundingI:
                surroundingI.append(currIndex)

    possible = []
    for i in surroundingI:
        for bus in grid[i]:
            possible.append(PLUTOArray[int(bus)])

    angle = 90 - heading
    if heading < 0:
        heading += 360
    if heading > 360:
        heading -= 360

    result = findDoorCoords(camCoord, angle, possible)
    toLL = (result[0], result[1], 18, "T")
    return utm.to_latlon(*toLL)


if __name__ == "__main__":
    # Calculate the computation time
    start = time.time()

    # camCoord = [40.74177025108547, -73.99407876316269]
    camCoord = [40.7665674, -73.9792634]
    # heading = -130.45825298954654
    heading = 290
    print("From origin calculator")
    print(main(camCoord, heading))

    # Calculate the computation time
    end = time.time()
    print("Time taken: ", end - start)
