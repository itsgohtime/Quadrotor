settings {
    nodaemon = true,
    statusInterval = 1
}

sync {
    default.rsync,
    source = "/home/itsgohtime/Documents/quadrotor",
    target = "pi@192.168.0.1:/home/pi/flight_controller",
    delay = 1, 
    exclude = { '/.git', '/.idea' }
}   