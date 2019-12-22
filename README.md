# RS485-master-slave
Realize master-slave communication using RS485 connection. Communication protocol is a simplified version of modbus.

## Command format
| ID (1 byte) | Function code (1 byte) | Data (more bytes) |

### ID
- Master: 0
- slave: 1-255

### Function Code
- Write: 16(DEC)
- Read: 4(DEC)
- Note: function code in a command response from a slave is always the same as in the command.
### Data
- Data is formatted in byte, e.g., D0...D4

### Command: Send Parameter Update
| Slave ID | 16 | D0...D4 |

### Command: Send Parameter Update ACK
| 0 | 16 | ACK|
- ACK === 10(DEC)

### Command: Query Fan Status
| Slave ID | 4 |

### Command: Query Response
| 0 | 4 | Fan Status|
- Each slave corresponses to 3 Fans.
- Fan Status are 3 byte values D0...D2

## Retransmission
Retransmission number is set to **2**, i.e., at maximum Master will transmit **3** times per try.

For periodic query command, if Master cannot get a response of the Fan Status from Slave, it will mark 255 of the slave fan status, which can be regarded as a Error Code indicating that the slave device is disconnected.