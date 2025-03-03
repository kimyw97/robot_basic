/** @format */

import { SerialPort } from "serialport";
import { ReadlineParser } from "@serialport/parser-readline";

const port = new SerialPort({
  path: "/dev/ttyAMA0",
  baudRate: 115200,
});
const parser = port.pipe(new ReadlineParser({ delimiter: "\n" }));

port.on("open", () => {
  console.log("Serial Port Opened");
});

parser.on("data", (data) => {
  console.log("Received:", data);
});

port.write("Hello stm\n", (err) => {
  if (err) console.error("Error on write:", err);
});
