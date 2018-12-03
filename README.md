# cfsd-cognition-detectconelane
[![Build Status](https://travis-ci.org/cfsd/cfsd18-cognition-detectconelane.svg?branch=master)](https://travis-ci.org/cfsd/cfsd18-cognition-detectconelane)


### How it works
- od4.dataTrigger: if recieving cone information, trigger collector.CollectorCones(); if recieving geolocation, trigger detectconelane.nextPos()
- nextPos() will extract the global position (with longitude and latitude), and make some adjustment with gpsReference
- collector will gather all cones in each frame, and then pass the frame to detectorlane
- then cones are seperated into four categories: coneLeft, coneRight, coneSmall, coneBig; based on the corresponding type: blue, yellow, small orange, big orange
- sort coneLeft and coneRight by the distance between cone and vehicle, from closest to furthest
- compute the path length for each side by adding up the distance of adjacent cones, and decide which side has longer path
- for the shorter path, insert some needed guessed cones if slam is not activated or the number of cones is not enough
- segment the lane into many small surfaceArea: divide the long side path with a specific number of equidistant points, and then find out the corresponding point on the short side of each virtual point on the long side
- send out surfaceArea via od4.send; the surfaceArea has x1,y1,x2,y2,x3,y3,x4,y4, corresponding to the four points in each segmentation


### OD4Session message in and out
- recieve:
  - opendlv.logic.perception.ObjectDirection
  - opendlv.logic.perception.ObjectDistance
  - opendlv.logic.perception.ObjectType
    - sender: (perception-detectcone, 118), (sensation-slam, 120), (sim-cognition-detectcone, 231)
  - opendlv.logic.sensation.Geolocation
    - sender: (proxy-ellipse2n, 112)
- send:
  - opendlv.system.SignalStatusMessage
  - opendlv.logic.perception.GroundSurfaceArea (x1,y1, x2,y2, x3,y3, x4,y4)
    - sender: (cognition-detectconelane, 211)


### Command line arguments
| parameter | comments |
| ----- | ----- |
| cid | OpenDaVINCI v4 session identifier |
| verbose | if true, provide lots of logging output |
| id | sender stamp of this microservice (211) |
| detectConeId | sender stamp of perception-detectcone (118) |
| slamId | sender stamp of sensation-slam (120) |
| simDetectConeId | sender stamp of sim-cognition-detectcone (231) |
| attentionId | sender stamp of sensation-attention (116) |
| yawRateId | sender stamp of proxy-ellipse2n (112) |
| speedId1 | wheelEncoderIdLeft |
| speedId2 | wheelEncoderIdRight |
| gpsId | sender stamp of perception-ukf (114) |
| gatheringTimeMs | |
| separationTimeMs | |
| alwaysSlam | |
| guessDistance | |
| minGuessSeparation | |
| latePerpGuessing | |
| useCurveDetection | |
| maxConeAngle | |
| behindMemoryDistance | |
| maxConeWidthSeparation | |
| widthSeparationMargin | |
| maxConeLengthSeparation | |
| lengthSeparationMargin | |
| nLapsToGo | |
| lapCounterLockTime | |
| stopSignalDelay | |
| useOrangeLapCounter | |
| useNewConeLapCounter | |
| nMinOrangeFrames | |
| nMinDisappearanceFrames | |
| useGpsLapCounter | |
| useRawGPS | |
| refLatitude | |
| refLongitude | |
| useAccelerationMode | |
| usePathMemory | |
| useSkidpadMode | |
- example usage:
    cfsd18-cognition-detectconelane --cid=${CID} --verbose=0 --id=211 --detectConeId=118 --slamId=120 --simDetectConeId=231 --attentionId=116 --yawRateId=112 --speedId1=1504 --speedId2=1505 --gpsId=114 --gatheringTimeMs=50 --separationTimeMs=10 --alwaysSlam=0 --guessDistance=3.2 --minGuessSeparation=1.5 --latePerpGuessing=0 --useCurveDetection=0 --maxConeAngle=1.570796325 --behindMemoryDistance=3.0 --maxConeWidthSeparation=3.2 --widthSeparationMargin=1.0 --maxConeLengthSeparation=5.0 --lengthSeparationMargin=1.0 --nLapsToGo=3 --lapCounterLockTime=20 --stopSignalDelay=0.0 --useOrangeLapCounter=1 --useNewConeLapCounter=1 --nMinOrangeFrames=2 --nMinDisappearanceFrames=2 --useGpsLapCounter=1 --useRawGPS=0 --refLatitude=57.710482 --refLongitude=11.950813 --useAccelerationMode=0 --usePathMemory=0 --useSkidpadMode=0


### Function call graph
- main()
  - od4.dataTrigger()
    - collector.CollectionCones()  [if recieve ObjectDirection or ObjectionDistance or ObjectType]
      - [thread] collector.InitializeCollection()
        - collector.GetCompleteFrame()
        - collector.SendFrame()
          - detectconelane.receiveCombinedMessage()
            - detectconelane.sortIntoSideArrays()
              - detectconelane.generateSurfaces()
                - detectconelane.orderAndFilterCones() or .orderCones()
                - detectconelane.findTotalPathLength()
                - detectconelane.insertNeededGuessedCones()
                  - detectconelane.guessCones()
                - detectconelane.findSafeLocalPath()
                  - detectconelane.findTotalPathLength()
                  - detectconelane.placeEquidistantPoints()
                  - detectconelane.findFactorToClosestPoint()
                  - detectconelane.sendMatchedContainer()
                    - od4.send()
    - detectconelane.nextPos() [if recieve Geolocation]


### Questions
- when segmenting the short side path, we should check "if one of the two segments next to the cone has a perpendicular line to the long side point. If so, place the short side point on that place of the segment"; what does perpendicular to long side point mean?
- ..
