const express = require('express')
const { SerialPort, ReadlineParser } = require("serialport");
const Readline = require("@serialport/parser-readline");
var bodyParser = require('body-parser');
const app = express();
const port = 80;

// Define serial port
const sPort = new SerialPort({ path: 'COM3', baudRate: 115200 })

// Serial port parser
const parser = new ReadlineParser();
sPort.pipe(parser);

// Read serial port data
let serialData = "";
parser.on('data', (data) => {
    console.log(data);
    serialData = data;
});

app.use('/', express.static('public'));

app.use(bodyParser.urlencoded({ extended: true }))
app.use(bodyParser.json())

// Handle client-sent events
app.post('/client-sent-events', function(req, res) {
    const clientData = req.body.data;

    if (clientData) {
        console.log(`Received data from client: ${clientData}`);
        // Optionally write data to serial port
        sPort.write(clientData, (err) => {
            if (err) {
                return res.status(500).send({ message: 'Failed to write to serial port' });
            }
            res.send({ message: 'Data sent to serial port successfully' });
        });
    } else {
        res.status(400).send({ message: 'No data received' });
    }
});

app.get('/server-sent-events', function(req, res) {

    res.writeHead(200, {
        'Content-Type': 'text/event-stream',
        'Cache-Control': 'no-cache',
        'Connection': 'keep-alive'
    });

    var interval = setInterval(function(){
        res.write("data: " + serialData + "\n\n");
    }, 1500);

    // close
    res.on('close', () => {
        clearInterval(interval);
        res.end();
    });
})





app.listen(port, () => {
  console.log(`Listening at http://localhost:${port}`)
})