/*********  lowest-level I2C transfers ***********/

/** Write value to this register on this I2C device. */
function i2cWriteByte(device: number, register: number, value: number) {
    let twoBytes = pins.createBuffer(2)
    twoBytes[0] = register
    twoBytes[1] = value
    pins.i2cWriteBuffer(device, twoBytes, true)
    control.waitMicros(100)
}

/**  Read byte from a register on this I2C device. */
function i2cReadByte(device: number, register: number) {
    pins.i2cWriteNumber(device, register, NumberFormat.UInt8LE) // select register
    return pins.i2cReadNumber(device, NumberFormat.UInt8LE, false) // read and return
}

/** Read a Buffer (array of bytes) from this I2C device, starting from given register. */
function i2cReadBuffer(device: number, register: number, length: number): Buffer {
    let buffer = pins.createBuffer(length)
    pins.i2cWriteNumber(device, register, NumberFormat.UInt8LE)
    buffer = pins.i2cReadBuffer(device, length, false)
    return buffer
}

/** Write a Buffer (array of bytes) to this I2C device, starting from given register. */
function i2cWriteBuffer(device: number, register: number, data: Buffer) {
    pins.i2cWriteNumber(device, register, NumberFormat.UInt8LE)
    pins.i2cWriteBuffer(device, data, false)
}

/** Read an array of [count] big-endian words from this I2C device, starting from given register. */
function i2cReadWordsBE(device: number, register: number, count: number): number[] {
    let buffer = i2cReadBuffer(device, register, 2 * count)
    let vals = buffer.toArray(NumberFormat.Int16BE)
    return vals
}

/** Read an array of [count] little-endian words from this I2C device, starting from given register. */
function i2cReadWordsLE(device: number, register: number, count: number): number[] {
    let buffer = i2cReadBuffer(device, register, 2 * count)
    let vals = buffer.toArray(NumberFormat.Int16LE)
    return vals
}

/** Modify flags in a register on this I2C device */
function i2cAdjustFlags(device: number, register: number, unsetMask: number, setMask: number) {
    let setting = i2cReadByte(device, register)
    setting &= (0xff ^ unsetMask)
    setting |= setMask
    i2cWriteByte(device, register, setting)
    control.waitMicros(10)
}