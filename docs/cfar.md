# Constant False Alarm Rate (CFAR)
Since an incoming signal is constantly changing, the threshold, $V_T$, must also constantly update to classify parts of the signal as targets with the same false alarm rate.

**Common techniques:**

**Cell averaging** - The mean power is taken of all the cells surrounding the cell under test (CUT), by the following equation:

$$
    \begin {align}
        Z &= \frac{1}{N}\sum_{n=1}^{N}X_n \\ 
        N &- \text{number of range cells being averaged} \\
        X_n &- \text{signal power of } n^{th} \text{cell} \\
        Z &- \text{local noise power estimate}
    \end {align}
$$

**Greatest** - This is the same as the cell averaging technique but it only uses the nearby cells on the side of the CUT that have the larger mean.

**Smallest** - Very similar to the greatest technique but it takes the nearby cells with the smaller of the two means. 

**Ordered statistic** - *NOT IMPLEMENTED*

**Cell averaging Statistic Hofele** - *NOT IMPLEMENTED*

To reduce interference from the CUT, nearby cells are omitted from the mean power and are called *guard cells*. 

To determine if a cell will be considered a target, the cell's value, $X$, must be greater than the average of the nearby cells multiplied by a bias factor, $C$. In other words, $X \ge CZ$ for a cell to be classified as a target.