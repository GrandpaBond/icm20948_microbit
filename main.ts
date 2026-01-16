
// ICM20948 library uses trace.ts calls, which in turn use the datalogger extension.
// Trace messages are logged only if their <level> bit is set in this global:
//let TRACE_LEVEL = 0b11111
/* Levels are as follows:
2**0 =  1 : ICM Chip Debug (Outputs library main messages)
2**1 =  2 : DMP Chip debug (Outputs DMP memory accesses)
2**2 =  4 : DMP FIFO processing debug (Outputs values during FIFO processing)
2**3 =  8 : DMP Output debug (Outputs values after FIFO process)
2**4 = 16 : DMP Register / I2C debug(Outputs bus communication)
2**5 = 32 : Low - level debug
*/

TRACE_LEVEL = 1

serial.redirectToUSB()
datalogger.mirrorToSerial(true) // monitor Log output

let icm = new ICM20948() // default to RAW mode

//basic.showNumber(icm.status)
pause(1000)

if (icm.status == 3) {
    basic.showIcon(IconNames.Happy)
} else {
    basic.showIcon(IconNames.Sad)
}
pause(1000)
basic.clearScreen()
basic.showString('shake everywhere')
icm.calibrateAll(true, 10000)
trace(1, 'calibration bias offsets')
datalogger.log(
    datalogger.createCV("AX", icm.accBias[0]),
    datalogger.createCV("AY", icm.accBias[1]),
    datalogger.createCV("AZ", icm.accBias[2]),
    datalogger.createCV("GX", icm.gyrBias[0]),
    datalogger.createCV("GY", icm.gyrBias[1]),
    datalogger.createCV("GZ", icm.gyrBias[2]),
    datalogger.createCV("MX", icm.magBias[0]),
    datalogger.createCV("MY", icm.magBias[1]),
    datalogger.createCV("MZ", icm.magBias[2]))

pause(1000)


enum Tests {
    TEST,
    SENSE,
    ACCEL,
    GYRO,
    MAG
}
let testsOn = ['T', 'S', 'A', 'G', 'M']
let testsOff = ['t', 's', 'a', 'g', 'm']

let accelData: number[]
let gyroData: number[]
let magData: number[]

// select sensor to test
let test = 0
basic.showString(testsOff[test])
pause(1000)
basic.clearScreen()
let active = false

input.onButtonPressed(Button.AB, function () {
    // clear down the log-file
    datalogger.deleteLog()
    datalogger.includeTimestamp(FlashLogTimeStampFormat.None)
    basic.showIcon(IconNames.Scissors)
})

// Button A cycles to next testsOff[] state
input.onButtonPressed(Button.A, function () {
    active = false
    test = (test + 1) % testsOn.length
    basic.showString(testsOff[test])
    //pause(1000)
    //basic.clearScreen()
})

// Button B toggles current test on/off
input.onButtonPressed(Button.B, function () {
    if (active) {
        active = false
        basic.showString(testsOff[test])
    } else {
        active = true
        basic.showString(testsOn[test])
    }
})

trace(1, 'STARTING UP')

while (true) {
    if (active) {
        switch (test) {
            case Tests.TEST:
                basic.showString('?', 75)
                basic.pause(500)

                break
            case Tests.SENSE:
                accelData = icm.readAcc()
                gyroData = icm.readGyr()
                magData = icm.readMag()
                datalogger.log(
                    datalogger.createCV("AX", accelData[0]),
                    datalogger.createCV("AY", accelData[1]),
                    datalogger.createCV("AZ", accelData[2]),
                    datalogger.createCV("GX", gyroData[0]),
                    datalogger.createCV("GY", gyroData[1]),
                    datalogger.createCV("GZ", gyroData[2]),
                    datalogger.createCV("MX", magData[0]),
                    datalogger.createCV("MY", magData[1]),
                    datalogger.createCV("MZ", magData[2]))
                break
            case Tests.ACCEL:
                accelData = icm.readAcc()
                datalogger.log(
                    datalogger.createCV("AX", accelData[0]),
                    datalogger.createCV("AY", accelData[1]),
                    datalogger.createCV("AZ", accelData[2]))
                break
            case Tests.GYRO:
                gyroData = icm.readGyr()
                datalogger.log(
                    datalogger.createCV("GX", gyroData[0]),
                    datalogger.createCV("GY", gyroData[1]),
                    datalogger.createCV("GZ", gyroData[2]))
                break
            case Tests.MAG:
                gyroData = icm.readMag()
                datalogger.log(
                    datalogger.createCV("MX", gyroData[0]),
                    datalogger.createCV("MY", gyroData[1]),
                    datalogger.createCV("MZ", gyroData[2]))
                break
        }
        basic.clearScreen()
        pause(200)
        //basic.showString(testsOn[test])
    } else {
        basic.clearScreen()
        pause(100)
        basic.showString(testsOff[test])
    }
}
