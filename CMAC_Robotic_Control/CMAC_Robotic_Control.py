from MainScripts import *

# Author: Alastair Poole
# Email: alastair.poole@strath.ac.uk
# Entry point to the library.



# TO DO :
# LBR to KMP transformation calibration


# Later TODO:
# Integrate into V-rep sim environment,
# Additional functions.



# connect to robot:
# receive info:
rIP = '172.31.1.137'
rP = 30001
# send info:
sIP = '172.31.1.10'
sP = 30010

# Chessboard data:
Ncols = 8
Nrows = 6
size = 24#[mm]


if __name__ == "__main__":
    rc = General(rIP,rP,sIP,sP)

    # run saved motion batches:
    rc.RunBatches()

    # Make sure to exit so the robot's threads are cut 
    # - causes an error Java side if you don't and you'll need to re-upload project before continuing.
    rc.Exit()

    # set the task sequences - instructions are included when you run it.
    rc.SetTaskSequencing()

    # calibrate camera with pre-set positions:
    rc.CameraCalibration(Nrows,Ncols,size)
    rc.Exit()
