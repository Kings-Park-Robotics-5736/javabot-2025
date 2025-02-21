import json
import math

#create a json object with the below json
#json object
#json object
samplePath = """
{
  "version": "2025.0",
  "waypoints": [
    {
      "anchor": {
        "x": 2.75,
        "y": 3.953
      },
      "prevControl": null,
      "nextControl": {
        "x": 2.9999017509951558,
        "y": 3.9600081987382723
      },
      "isLocked": false,
      "linkedName": null
    },
    {
      "anchor": {
        "x": 3.194,
        "y": 3.953
      },
      "prevControl": {
        "x": 2.9440699268876624,
        "y": 3.947087424075617
      },
      "nextControl": null,
      "isLocked": false,
      "linkedName": null
    }
  ],
  "rotationTargets": [],
  "constraintZones": [],
  "pointTowardsZones": [],
  "eventMarkers": [],
  "globalConstraints": {
    "maxVelocity": 3.0,
    "maxAcceleration": 3.0,
    "maxAngularVelocity": 540.0,
    "maxAngularAcceleration": 720.0,
    "nominalVoltage": 12.0,
    "unlimited": false
  },
  "goalEndState": {
    "velocity": 0,
    "rotation": 90.0
  },
  "reversed": false,
  "folder": null,
  "idealStartingState": {
    "velocity": 0.25,
    "rotation": 90.0
  },
  "useDefaultConstraints": true
}
"""

#parse the json object
samplePath = json.loads(samplePath)

aprilTags = """
160.39 130.17 12.13 240 0 "CD"
144.00 158.50 12.13 180 0 "AB"
160.39 186.83 12.13 120 0 "LK"
193.10 186.83 12.13 60 0 "JI"
209.49 158.50 12.13 0 0 "HG"
193.10 130.17 12.13 300 0 "FE"
"""

lines = aprilTags.strip().split('\n')
path_data = []

for line in lines:
    parts = line.split()
    path_data.append({
        "x": float(parts[0]),
        "y": float(parts[1]),
        "z": float(parts[2]),
        "rotation": int(parts[3]),
        "none": int(parts[4]),
        "name": parts[5].strip('"')
    })

print(path_data)
bumperWidth = 36.5
offsetShort = 2.7
offsetLong = 15.7
offsets = [offsetShort, offsetLong]
offsetsL23 = [-offsetLong - 1, -offsetShort]
names = ["Left", "Right"]
for tag in path_data:
    robotRotationL4 = tag['rotation'] - 90
    robotRotationL23 =  tag['rotation'] -90-180
    for i in range(0,2):
        robotPositionX = tag['x'] + math.cos(math.radians(tag['rotation']))*(bumperWidth*.5)+math.cos(math.radians(360 - tag['rotation'] - 90))*offsets[i]
        robotPositionY = tag['y'] + math.sin(math.radians(tag['rotation']))*(bumperWidth*.5)-math.sin(math.radians(360 - tag['rotation'] - 90))*offsets[i]
        robotPositionX *= 0.0254
        robotPositionY *= 0.0254
        startX = robotPositionX + .5 *math.cos(math.radians(tag['rotation']))
        startY = robotPositionY + .5 *math.sin(math.radians(tag['rotation']))

        controlX = startX + .25  * math.sin(math.radians(robotRotationL4))
        controlY = startY - .25 * math.cos(math.radians(robotRotationL4))


        print(tag['name'] + " " + names[i] + " " + str(robotPositionX) + " " + str(robotPositionY) + " "  + str(robotRotationL4) + ", start: " + str(startX) + " " + str(startY))
        # in samplePath, replace the first waypoint with the new waypoint startX, startY, robotRotationL4
        samplePath['waypoints'][0]['anchor']['x'] = startX
        samplePath['waypoints'][0]['anchor']['y'] = startY
        samplePath['waypoints'][0]['nextControl']['x'] = controlX
        samplePath['waypoints'][0]['nextControl']['y'] = controlY
        samplePath['waypoints'][0]['linkedName'] = "SCORE" + tag['name'] + "L4" + names[i] + "START"

        #in samplePath, replace the second waypoint with the new waypoint robotPositionX, robotPositionY, robotRotationL4
        samplePath['waypoints'][1]['anchor']['x'] = robotPositionX
        samplePath['waypoints'][1]['anchor']['y'] = robotPositionY
        samplePath['waypoints'][1]['prevControl']['x'] = controlX
        samplePath['waypoints'][1]['prevControl']['y'] = controlY
        samplePath['waypoints'][1]['linkedName'] = "SCORE" + tag['name'] + "L4" + names[i] + "END"


        #put the rotation L4 in the goalEndState and idealStartingState
        samplePath['goalEndState']['rotation'] = robotRotationL4
        samplePath['idealStartingState']['rotation'] = robotRotationL4
        

        #write the json object with the format tag[name] + "L4" + names[i] + ".json"
        with open(tag['name'] + "L4" + names[i] + "GEN.path", "w") as f:
            json.dump(samplePath, f, indent=4)

    for i in range(0,2):
        robotPositionX = tag['x'] + math.cos(math.radians(tag['rotation']))*(bumperWidth*.5)+math.cos(math.radians(360 - tag['rotation'] - 90))*offsetsL23[i]
        robotPositionY = tag['y'] + math.sin(math.radians(tag['rotation']))*(bumperWidth*.5)-math.sin(math.radians(360 - tag['rotation'] - 90))*offsetsL23[i]
        robotPositionX *= 0.0254
        robotPositionY *= 0.0254
        startX = robotPositionX + .5 *math.cos(math.radians(tag['rotation']))
        startY = robotPositionY + .5 *math.sin(math.radians(tag['rotation']))

        controlX = startX + .25  * math.sin(math.radians(robotRotationL23+180))
        controlY = startY - .25 * math.cos(math.radians(robotRotationL23+180))


        print(tag['name'] + " " + names[i] + " " + str(robotPositionX) + " " + str(robotPositionY) + " "  + str(robotRotationL23) + ", start: " + str(startX) + " " + str(startY))
        # in samplePath, replace the first waypoint with the new waypoint startX, startY, robotRotationL4
        samplePath['waypoints'][0]['anchor']['x'] = startX
        samplePath['waypoints'][0]['anchor']['y'] = startY
        samplePath['waypoints'][0]['nextControl']['x'] = controlX
        samplePath['waypoints'][0]['nextControl']['y'] = controlY
        samplePath['waypoints'][0]['linkedName'] = "SCORE" + tag['name'] + "L23" + names[i] + "START"

        #in samplePath, replace the second waypoint with the new waypoint robotPositionX, robotPositionY, robotRotationL4
        samplePath['waypoints'][1]['anchor']['x'] = robotPositionX
        samplePath['waypoints'][1]['anchor']['y'] = robotPositionY
        samplePath['waypoints'][1]['prevControl']['x'] = controlX
        samplePath['waypoints'][1]['prevControl']['y'] = controlY
        samplePath['waypoints'][1]['linkedName'] = "SCORE" + tag['name'] + "L23" + names[i] + "END"


        #put the rotation L4 in the goalEndState and idealStartingState
        samplePath['goalEndState']['rotation'] = robotRotationL23
        samplePath['idealStartingState']['rotation'] = robotRotationL23
        

        #write the json object with the format tag[name] + "L4" + names[i] + ".json"
        with open(tag['name'] + "L23" + names[i] + "GEN.path", "w") as f:
            json.dump(samplePath, f, indent=4)

