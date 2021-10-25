var msg1 = { payload: msg.payload.length };
msg1.payload = msg.payload.uplink_message.decoded_payload.Ibatt;

var msg2 = { payload: msg.payload.length };
msg2.payload = msg.payload.uplink_message.decoded_payload.Vbatt;

var msg3 = { payload: msg.payload.length };
msg3.payload = msg.payload.uplink_message.decoded_payload.Ppower;

var msg4 = { payload: msg.payload.length };
msg4.payload = msg.payload.uplink_message.decoded_payload.Iload;

var msg5 = { payload: msg.payload.length };
msg5.payload = msg.payload.uplink_message.decoded_payload.CS;

var msg6 = { payload: msg.payload.length };
msg6.payload = msg.payload.uplink_message.decoded_payload.LoadS;

var msg7 = { payload: msg.payload.length };
msg7.payload = msg.payload.uplink_message.decoded_payload.yieldtotal;

var msg8 = { payload: msg.payload.length };
msg8.payload = msg.payload.uplink_message.decoded_payload.yieldday;

var msg20 = {};
msg20.payload = [{"Ibatt": msg1.payload, "Vbatt": msg2.payload, "Ppower": msg3.payload, "Iload": msg4.payload, "CS": msg5.payload, "LoadState": msg6.payload, "yieldT": msg7.payload, "yieldD": msg8.payload}];

return [msg20];
