function Decode(fPort, bytes, variables) {
  var i = 0;
  var decoded = {};
  
  if (fPort === 1) {
    // decode payload for port 1
    decoded.pirValue = bytes[i++] | (bytes[i++] << 8);
    decoded.mq135Value = bytes[i++] | (bytes[i++] << 8);
    decoded.mq7Value = bytes[i++] | (bytes[i++] << 8);
    decoded.dust = bytes[i++] | (bytes[i++] << 8);
    
    // add support for Class B fields
    if (variables.SupportsClassB) {
      decoded.ClassBTimeout = bytes[9] * 60; // convert from minutes to seconds
      decoded.PingSlotPeriod = bytes[10] & 0x0F; // first 4 bits represent ping slot period
      decoded.PingSlotDR = bytes[10] >> 4 & 0x0F; // next 4 bits represent ping slot data rate
      decoded.PingSlotFreq = bytes[11] << 16 | bytes[12] << 8 | bytes[13]; // frequency in Hz
    }
    
    // add support for Class C fields
    if (variables.SupportsClassC) {
      decoded.ClassCTimeout = bytes[9] * 60; // convert from minutes to seconds
    }
    
    // add support for other fields
    decoded.MACVersion = bytes[14] >> 4;
    decoded.RegParamsRevision = bytes[14] & 0x0F;
    decoded.SupportsJoin = bytes[15] & 0x80;
    decoded.RXDelay1 = bytes[16];
    decoded.RXDROffset1 = bytes[17] >> 4;
    decoded.RXDataRate2 = bytes[17] & 0x0F;
    decoded.RXFreq2 = (bytes[18] << 16 | bytes[19] << 8 | bytes[20]) / 100;
    decoded.FactoryPresetFreqs = bytes[21] & 0x3F; // last 6 bits represent number of factory-preset frequencies
    decoded.MaxEIRP = (bytes[22] << 8 | bytes[23]) / 100;
  }
  
  return decoded;
}
