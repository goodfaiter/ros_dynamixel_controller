from dynamixel_easy_sdk import Connector

def scan_dynamixel_motors(logger):
    """
    Scanner that directly uses the motor ping function
    """
    # Common baud rates for Dynamixel
    baud_rates = [9600, 57600, 1000000, 115200, 2000000, 3000000, 4000000]
    
    for baud_rate in baud_rates:
        logger.info(f"Trying baud rate: {baud_rate}")
        
        # Create connector with current baud rate
        connector = Connector("/dev/ttyUSB0", baud_rate)

        try:
            motor_ids = connector.broadcastPing()
            if motor_ids:
                for id in motor_ids:
                    logger.info(f"Found motor ID: {id}")
        
        except Exception as e:
            logger.error(f"Error at baud rate {baud_rate}: {e}")
            
        connector.closePort()