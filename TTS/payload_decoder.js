function decodeUplink(input) {
    var data = {};

    data.Vbatt = (input.bytes[0]  <<8 | input.bytes[1]) / 100;
    data.Ibatt = ((input.bytes[2] & 0x80 ? 0xFFFF<<16 : 0) | input.bytes[2]<<8 | input.bytes[3]) / 100;
    data.Iload = (input.bytes[4]  <<8 | input.bytes[5]) / 10;
    data.Ppower = input.bytes[6]; 
    data.CS = input.bytes[7]; 
    data.LoadS = input.bytes[8];
    data.yieldtotal = (input.bytes[9]  <<8 | input.bytes[10]) * 10;
    data.yieldday = input.bytes[11] * 10;
  
    if (data.Vbatt > 0){
        return {
            data: data,
            warnings: [],
            errors: []
        };
    }
}
