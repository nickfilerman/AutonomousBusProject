# Team 2 Project Proposal: Nick Filerman &amp; Stephen Ng

- Describe the customer&#39;s automated mobility task that you are supporting. What SAE level is it. What are the benefits and risks to an automated solution?
  - We are going to implement support for the SAE level 4 autonomous bus, that counts the number of the people currently waiting at a bus stop. It then takes that data, combines it with the current time of day, as well as if someone has requested to stop, and then decides whether to stop at that given bus stop.
- Describe the performance or safety aspect that you are addressing; what is the risk or problem you are addressing
  - The performance aspect we are addressing can significantly reduce the time taken to traverse a bus route, by passing through stops that the bus does not need to stop at.
  - One problem we may run into is determining the distance a person needs to be from the bus stop to consider them &quot;at the bus stop&quot;.
- Explain what your system does including what analysis, testing or optimization it performs. If your system works as intended, what benefits will it provide?
  - Will improve efficiency of bus route
  - We will have to manually test this, through different ways, one way would be to place a flag over the video that shows whether the bus is set to stop at a given bus stop. True for stopping, false for passing.
  - This is common practice for SAE level 0 busses, as it makes the bus route much quicker. Implementing this for the autonomous bus should prove the same results
  - Our primary goal is to split up the tasks as much as possible into multiple functions, for ease of use, as well as possible usage of each smaller function down the line, such as counting pedestrians. In doing so, it will make our solution much more readable and easier to work with. Our primary function, will then return a boolean value to determine whether the bus should stop, based off given conditions, that can be manipulated.
- Give illustrative and/or quantitative results from your system. How well does it work?
  - The bus should stop at a given bus stop if one of these conditions are met, otherwise it should continue with the bus route, passing by the bus stop:
    - There is at least 1 person waiting at the bus stop
    - The time of day is before 7pm
    - Someone has requested a stop
  - Our primary function should just return a boolean value that tells the bus whether to stop.
