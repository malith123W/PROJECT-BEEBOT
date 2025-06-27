import { connectToServer, serverData, pingAck, ping, setPingAck, resetServerData } from './mqttClient'



let icons = ['fas fa-globe-americas', 'fas fa-globe-europe', 'fas fa-globe-africa', 'fas fa-globe-asia']
let count = 0;
var online = false
var serverIdx = 0; //the index of the server that needed to be connected
export let serverList = {}
let pingFailures = 0;
const MAX_PING_FAILURES = 3;

export function animateInternetIcon() {

    if (online) {
        setTimeout(() => {
            document.getElementById('InternetIcon').style.color = '#21f873'
            
            document.getElementById("online").style.color = '#21f873'
            document.getElementById("online").textContent = "online"

            document.getElementById("InternetIcon").className = icons[count]
            count++;
            count = count % 4;
            animateInternetIcon();
        }, 1000)
    }else{
        document.getElementById('InternetIcon').style.color = '#9da7a1'
        document.getElementById("online").style.color = '#9da7a1'
        document.getElementById("online").textContent = "offline"

    }
}

export function setOnline() {
    // This function is currently unused. Remove or implement as needed.
}

export function findServer() {
    //search for available servers
    if (Object.keys(serverList).length == 0) {
        console.log("ServerList Empty");
        updateServerList();
        document.getElementById('serverName').textContent = 'No Servers'
        setTimeout(findServer, 2000);
    } else {
        if (serverData != null) {
            console.log("findServer: serverData is set, going online");
            document.getElementById('serverName').textContent = serverList[serverIdx][1]
            online = true;
            pingServer()
            animateInternetIcon()
        } else {
            // connect to a specified server by the index, following is set to "0" for now
            connectToServer(serverIdx);
            console.log("Trying to connect: "+serverList[serverIdx][1]);
            setTimeout(findServer, 2000);
        }
    }
}

// ping the server
function pingServer(){
    if(online){
        setTimeout(()=>{
            console.log(`pingAck: ${pingAck}, pingFailures: ${pingFailures}`);
            if (!pingAck) {
                pingFailures++;
                console.log(`Missed ping! Failures: ${pingFailures}`);
            } else {
                if (pingFailures > 0) console.log('Ping successful, resetting failures.');
                pingFailures = 0;
            }
            setPingAck(false);
            ping();
            if (pingFailures >= MAX_PING_FAILURES) {
                console.log('Too many missed pings, going offline.');
                online = false;
            }
            pingServer();
        }, 2000) // Increased interval to 2 seconds
    } else {
        console.log('Reconnecting due to ping failures...');
        pingFailures = 0;
        setPingAck(true);
        document.getElementById('serverName').textContent = "Reconnecting...";
        resetServerData();
        findServer();
    }
}

export function updateServerList(){
    Object.keys(serverList).forEach(key => delete serverList[key]);
    serverList[0] = [0, "Embedded Pro"];
}