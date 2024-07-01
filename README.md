# Embedded-System-Thermostat
TI CC3220X microcontroller simulated thermostat utilizing GPIO, UART2, and I2C peripherals.

# Summarize the project and the problem it was solving.
A customer, SysTec, wants to enter a lucrative market and has tasked your engineering department with creating a smart thermostat using the TI board. For the prototype, I used the TMP006 temperature sensor to read the room temperature (via I2C), an LED to indicate the output to the thermostat where "LED on = heat on" (via GPIO), two buttons to increase and decrease the set temperature (via GPIO interrupt), and the UART to simulate the data being sent to the server.
# What did you do particularly well?
I produced clear and concise code that can be recycled, providing many comments and industry best practices. Most embedded functionality was modularly created into individual reusable functions.
# Where could you improve?
Time management was the biggest issue during this project. I am a Sr. SYstem Engineer tasked with installing different software into a complex RHEL system and performing and learning various system administrator tasks. This often wore me out and negatively impacted my time researching embedded system programming concepts. Regardless of this challenge, I completed the project within a week of its due date. I can improve upon balancing other responsibilities with my current Computer Science endeavor.
# What tools and/or resources are you adding to your support network?
The concepts of a state machine put certain functionality into perspective. Additionally, learning how interfaces and peripherals work put many aspects of coding into perspective. Using an API and researching the API declarations helped me figure out how to learn to utilize a new API.
# What skills from this project will be particularly transferable to other projects and/or coursework?
The programming aspect's planning and breakdown will be transferable to many future projects. Also, knowing how to break down hardware functionality with bit programming deepens one's understanding of how embedded systems work.
# How did you make this project maintainable, readable, and adaptable?
Interfaces were broken down into reusable functions, with many comments explaining how each function works. These functions can easily be adjusted to fit other use cases in another project. This will also help encapsulate any bugs or errors in the thermostat's functionality allowing another team member to take over and maintain the product.
