# Near and Far Fields

It is important to have at least an idea of the behaviour of your received signal. 
This can be greatly influenced by the distance at which the targets you are measuring reside. 
The behaviour can be approximated by fitting the target into two main regions: near field and far field. 

Near field is the more difficult region because the spherical propagation pattern of the RF source must be taken into account. 
This causes the incident angle to vary non-linearly which makes angle of arrival (AoA) more difficult, but not impossible, to calculate. 
As the distance from the RF source increases, the spherical propagation pattern starts to resemble a plane wave. 
The far field is therefore the optimal region because it allows us to use a convienent approximation to relate the distance from the RF source to each element, $L$, and the AoA, $\theta$: $L=d\sin{\theta}$, where $d$ is the element separation.

The boundary between the two regions is defined by,

$$ 
    \begin {align}
        R &= \frac{2D^2}{\lambda} \\
    \end {align} 
$$

where $D$ is the antenna diameter and $\lambda$ is the wavelength. 
For a uniform linear array the effective diameter is $D=d(N-1)$ where $N$ is the number of elements.

As an example, the Analog Devices' *FMCW Phaser Board* is taken with the following parameters:

Parameter | Description | Value | Unit
---|---|---|---|
$f_c$ | Center frequency | 12.1 | GHz
$\lambda$ | Wavelength | 24.8 | mm
$N$ | Number of elements | 8 | -
$d$ | Element separation | 15 | mm

Using the equations above, the effective antenna diameter would be $D=105$ mm and the field boundary would be $R = 889$ mm. 


