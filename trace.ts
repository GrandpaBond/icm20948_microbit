
/** if <level> bit in global TRACE_LEVEL, log message built from sequence of
 * <field>s and <val>s, as specified by sequence of <format> letters:
 * "S"=string; "N"=number; "B"=hexByte; "P"=hexPair; "Q"=hexQuad; "D"=hexData (from Buffer val)
 */
function trace(level: number, event: string, format: string = null,
    field0: string = null, val0: any = null,
    field1: string = null, val1: any = null,
    field2: string = null, val2: any = null,
    field3: string = null, val3: any = null) {
    if ((level & TRACE_LEVEL) > 0) {
        let forms = format + '----' // ensure at least 4 letters
        forms = forms.slice(0, 4)
        datalogger.log(
            datalogger.createCV("Event", event),
            datalogger.createCV(field0, show(val0, forms[0])),
            datalogger.createCV(field1, show(val1, forms[1])),
            datalogger.createCV(field2, show(val2, forms[2])),
            datalogger.createCV(field3, show(val3, forms[3])))
    }
}

function show(value: any, form: string): string {
    if ((value == null) || (form == '-')) return ''
    let valBuff = Buffer.create(20) // seems to be needed for Buffer supplied as "any"
    let valType = typeof value
    let combo = valType + form
    switch (combo) {
        case 'numberN':
            return value // autoconvert to string
            break
        case 'numberB': return hexByte(value) // 8-bit
            break
        case 'numberP': return hexPair(value) // 16-bit (BE)
            break
        case 'numberQ': return hexQuad(value) // 32-bit (BE)
            break
        case "stringS": // simple string
            return value
            break
        case "stringD": // convert string to a Buffer, then show bytes in hex
            valBuff = stringToBuffer(value)
            return hexBuffer(valBuff, 0, valBuff.length)
            break
        case 'objectS': // presumably a Buffer: stringify it
            valBuff = value
            return valBuff.toString()
            break
        case "objectD": // presumably a Buffer: show bytes in hex
            valBuff = value
            return hexBuffer(value, 0, valBuff.length)
            break
        default:
            return ('format "' + form + '" incompatible with supplied ' + valType)
            break
    }
}


/** format byte value as "0xAB" */
function hexByte(byte: number): string {
    return '0x' + hexit(byte >> 4) + hexit(byte & 0xf)
}

/** format 16-bit value as "0xABCD" */
function hexPair(word: number): string {
    let hi = word >> 8
    let lo = word & 0xff
    return '0x' + hexit(hi >> 4) + hexit(hi & 0xf) + hexit(lo >> 4) + hexit(lo & 0xf)
}

/** format 32-bit value as "0xABCDEF01" */
function hexQuad(int: number): string {
    let hi = int >> 24 // byte 0
    let lo = (int >> 16) & 0xff // byte 1
    let s = '0x' + hexit(hi >> 4) + hexit(hi & 0xf) + hexit(lo >> 4) + hexit(lo & 0xf)
    hi = (int >> 8) & 0xff // byte 2
    lo = int & 0xff // byte 3
    s += hexit(hi >> 4) + hexit(hi & 0xf) + hexit(lo >> 4) + hexit(lo & 0xf)
    return s
}

/** format data in buffer as "01234567 89ABCD... ........" */
function hexBuffer(bytes: Buffer, from: number, count: number): string {
    let output = '0x'
    for (let i = 0; i < count; i++) {
        let byte = bytes.getNumber(NumberFormat.Int8LE, from + i)
        output += hexit(byte >> 4)
        output += hexit(byte & 0xf)
        if ((i % 4) == 3) output += ' 0x'
    }
    return output
}

/** convert nibble to hex digit character */
function hexit(nibble: number): string {
    const hex = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "A", "B", "C", "D", "E", "F"]
    return (hex[nibble & 0xf])
}

function stringToBuffer(str: string): Buffer {
    return Buffer.fromUTF8(str);
}