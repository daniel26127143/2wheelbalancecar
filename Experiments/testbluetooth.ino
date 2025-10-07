char val;

void setup() {
  Serial.begin(9600); //由於這個板子直接把RXD TXD連接在1,0所以直接使用Serial.print
  Serial.println("BT is ready!");
}

void loop() {
  if (Serial.available()) { //偵測有沒有輸入文字
    val = Serial.read();
    Serial.write(val); // 回傳使用者所輸入的字串
  }
}
