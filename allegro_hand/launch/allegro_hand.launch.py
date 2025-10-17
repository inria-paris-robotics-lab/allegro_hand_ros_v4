def setup_can(context):
    	can_port = context.launch_configurations['CAN_DEVICE']
    	commands = [
        	f"sudo ip link set {can_port} down",
        	f"sudo ip link set {can_port} type can bitrate 1000000",
        	f"sudo ip link set {can_port} up"
    	]
    
    	while True:
        	password = getpass.getpass('Enter sudo password: ')
        	success = True
        
        	for cmd in commands:
            		result = os.system(f'echo "{password}" | sudo -S {cmd}')
            		if result != 0:
                		print(f"Command failed: {cmd}")
                		success = False
                		break
        
        	if success:
            		print(f'{can_port} setup completed')
            		break
        	else:
            		print(f'{can_port} setup failed. Please try again.')
    
    	return []  


# obtenir le numero de serie de l'adaptateur : lsusb -v | grep -A 10 "PEAK-System"
