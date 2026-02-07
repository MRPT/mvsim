.. _world-flow-control:

World XML files flow control
--------------------------------------------

MVSim's XML world files support programming-like flow control structures to enable 
dynamic world generation, reduce code repetition, and create parametric scenarios.


For loops
============

The ``<for>`` tag allows you to repeat XML blocks with an iteration variable, similar to 
for-loops in programming languages.

Basic syntax
^^^^^^^^^^^^^^^

.. code-block:: xml

    <for var="variable_name" from="start_value" to="end_value">
        <!-- Content to repeat -->
    </for>

- **var**: Name of the iteration variable (can be dereferenced with ``${variable_name}``)
- **from**: Starting value (inclusive, integer)
- **to**: Ending value (inclusive, integer)

The loop body is executed for each value from ``from`` to ``to``, and the variable can be 
used in any XML attribute or content within the loop.


Simple example
^^^^^^^^^^^^^^^

Create a row of blocks:

.. code-block:: xml

    <for var="i" from="0" to="9">
        <block name="box_${i}">
            <init_pose>${i} 0 0</init_pose>
            <geometry type="box" lx="0.8" ly="0.8" lz="1.0"/>
        </block>
    </for>

This creates 10 blocks named ``box_0`` through ``box_9`` positioned at x-coordinates 0 through 9.


Using expressions with loop variables
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Loop variables can be used in mathematical expressions with the ``$f{...}`` notation:

.. code-block:: xml

    <for var="i" from="0" to="19">
        <block name="pole_${i}" class="lamp_post">
            <init_pose>$f{i*2.5} $f{sin(i*0.3)*5} 0</init_pose>
            <visual_scale>$f{0.8 + i*0.02}</visual_scale>
        </block>
    </for>

This creates 20 lamp posts with:

- Spacing of 2.5 meters along X-axis
- Sinusoidal Y-position variation
- Gradually increasing scale from 0.8 to 1.2


Nested loops
^^^^^^^^^^^^^^^

Loops can be nested to create 2D patterns:

.. code-block:: xml

    <for var="row" from="0" to="4">
        <for var="col" from="0" to="4">
            <block name="cube_${row}_${col}">
                <init_pose>$f{col*2} $f{row*2} 0</init_pose>
                <color>#FF0000FF</color>
                <geometry type="box" lx="1.5" ly="1.5" lz="1.5"/>
            </block>
        </for>
    </for>

This creates a 5×5 grid of colored cubes.


Advanced loop examples
^^^^^^^^^^^^^^^^^^^^^^^^^

Creating a circular arrangement:

.. code-block:: xml

    <variable name="NUM_ITEMS" value="12"/>
    <variable name="RADIUS" value="10.0"/>
    
    <for var="i" from="0" to="${NUM_ITEMS-1}">
        <block name="pillar_${i}" class="stone_pillar">
            <init_pose>
                $f{RADIUS*cos(i*2*M_PI/NUM_ITEMS)}
                $f{RADIUS*sin(i*2*M_PI/NUM_ITEMS)}
                $f{i*30}
            </init_pose>
        </block>
    </for>

Creating a parametric path:

.. code-block:: xml

    <for var="t" from="0" to="100">
        <block name="marker_${t}">
            <!-- Lissajous curve -->
            <init_pose>
                $f{5*sin(t*0.1)}
                $f{5*cos(t*0.15)}
                0
            </init_pose>
            <geometry type="cylinder" radius="0.1" length="0.5"/>
        </block>
    </for>


Conditional parsing
=======================

The ``<if>`` tag allows conditional inclusion of XML content based on a boolean condition.

Basic syntax
^^^^^^^^^^^^^^^

.. code-block:: xml

    <if condition="expression">
        <!-- Content included only if condition is true -->
    </if>

The **condition** attribute is evaluated as a boolean expression. The content inside the 
``<if>`` tag is only parsed and included if the condition evaluates to true.


Simple conditionals
^^^^^^^^^^^^^^^^^^^^^

Using variables:

.. code-block:: xml

    <variable name="ENABLE_DEBUG" value="true"/>
    
    <if condition="${ENABLE_DEBUG}">
        <block name="debug_marker">
            <init_pose>0 0 0</init_pose>
            <geometry type="sphere" radius="0.5"/>
        </block>
    </if>

Using mathematical expressions:

.. code-block:: xml

    <variable name="DIFFICULTY" value="2"/>
    
    <if condition="$f{DIFFICULTY >= 2}">
        <!-- Add extra obstacles for higher difficulty -->
        <block name="hard_obstacle" class="spike_trap">
            <init_pose>5 5 0</init_pose>
        </block>
    </if>


Combining with loops
^^^^^^^^^^^^^^^^^^^^^^^

Conditionals can be used inside loops for selective generation:

.. code-block:: xml

    <for var="i" from="0" to="20">
        <!-- Only create blocks at even positions -->
        <if condition="$f{i % 2 == 0}">
            <block name="even_block_${i}">
                <init_pose>${i} 0 0</init_pose>
                <geometry type="box" lx="1" ly="1" lz="1"/>
            </block>
        </if>
    </for>

Creating conditional patterns:

.. code-block:: xml

    <for var="x" from="-10" to="10">
        <for var="y" from="-10" to="10">
            <!-- Create a checkerboard pattern -->
            <if condition="$f{(x+y) % 2 == 0}">
                <block name="tile_${x}_${y}">
                    <init_pose>${x} ${y} 0</init_pose>
                    <color>#FFFFFF</color>
                    <geometry type="box" lx="0.9" ly="0.9" lz="0.1"/>
                </block>
            </if>
            <if condition="$f{(x+y) % 2 != 0}">
                <block name="tile_${x}_${y}">
                    <init_pose>${x} ${y} 0</init_pose>
                    <color>#000000</color>
                    <geometry type="box" lx="0.9" ly="0.9" lz="0.1"/>
                </block>
            </if>
        </for>
    </for>


Environment-based conditionals
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use environment variables for configuration:

.. code-block:: xml

    <!-- Enable sensors only in simulation mode -->
    <if condition="${SIM_MODE}">
        <vehicle name="robot1">
            <init_pose>0 0 0</init_pose>
            <!-- Include sensors -->
            <include file="sensors_full.xml"/>
        </vehicle>
    </if>
    
    <!-- Use simplified model for testing -->
    <if condition="$f{not ${SIM_MODE}}">
        <vehicle name="robot1">
            <init_pose>0 0 0</init_pose>
            <!-- No sensors in test mode -->
        </vehicle>
    </if>

Then run with:

.. code-block:: bash

    SIM_MODE=true mvsim myworld.world.xml


Combining flow control features
=================================

Complex world generation
^^^^^^^^^^^^^^^^^^^^^^^^^^

Combine loops, conditionals, and variables for sophisticated scenarios:

.. code-block:: xml

    <?xml version="1.0"?>
    <world>
        <!-- Configuration variables -->
        <variable name="ROOM_SIZE" value="20"/>
        <variable name="OBSTACLE_DENSITY" value="0.3"/>
        <variable name="ENABLE_BARRIERS" value="true"/>
        
        <!-- Grid of potential obstacle positions -->
        <for var="x" from="0" to="${ROOM_SIZE}">
            <for var="y" from="0" to="${ROOM_SIZE}">
                <!-- Randomly place obstacles based on density -->
                <if condition="$f{rand() < OBSTACLE_DENSITY}">
                    <block name="obstacle_${x}_${y}">
                        <init_pose>$f{x*2} $f{y*2} 0</init_pose>
                        <geometry type="box" 
                                  lx="$f{0.5 + rand()*1.0}" 
                                  ly="$f{0.5 + rand()*1.0}" 
                                  lz="$f{0.5 + rand()*2.0}"/>
                        <color>#$f{int(rand()*255):02X}$f{int(rand()*255):02X}$f{int(rand()*255):02X}</color>
                    </block>
                </if>
            </for>
        </for>
        
        <!-- Optional perimeter barriers -->
        <if condition="${ENABLE_BARRIERS}">
            <!-- North wall -->
            <block name="wall_north" static="true">
                <init_pose>$f{ROOM_SIZE} 0 0</init_pose>
                <geometry type="box" lx="0.5" ly="$f{ROOM_SIZE*2}" lz="3.0"/>
            </block>
            <!-- South wall -->
            <block name="wall_south" static="true">
                <init_pose>0 0 0</init_pose>
                <geometry type="box" lx="0.5" ly="$f{ROOM_SIZE*2}" lz="3.0"/>
            </block>
            <!-- East wall -->
            <block name="wall_east" static="true">
                <init_pose>0 $f{ROOM_SIZE} 0</init_pose>
                <geometry type="box" lx="$f{ROOM_SIZE*2}" ly="0.5" lz="3.0"/>
            </block>
            <!-- West wall -->
            <block name="wall_west" static="true">
                <init_pose>0 0 0</init_pose>
                <geometry type="box" lx="$f{ROOM_SIZE*2}" ly="0.5" lz="3.0"/>
            </block>
        </if>
    </world>


Parametric vehicle placement
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: xml

    <variable name="NUM_ROBOTS" value="5"/>
    <variable name="FORMATION" value="0"/>
    
    <for var="i" from="0" to="$f{NUM_ROBOTS-1}">
        <!-- Line formation -->
        <if condition="$f{FORMATION==0}">
            <vehicle name="robot_${i}" class="differential_robot">
                <init_pose>$f{i*3} 0 0</init_pose>
            </vehicle>
        </if>
        
        <!-- Circular formation -->
        <if condition="$f{FORMATION==1}">
            <vehicle name="robot_${i}" class="differential_robot">
                <init_pose>
                    $f{5*cos(i*2*M_PI/NUM_ROBOTS)}
                    $f{5*sin(i*2*M_PI/NUM_ROBOTS)}
                    $f{i*360/NUM_ROBOTS}
                </init_pose>
            </vehicle>
        </if>
    </for>


Best practices
================

Performance considerations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. **Minimize deep nesting**: Excessive nested loops can slow world parsing
2. **Use conditionals to skip unnecessary objects**: Filter early to avoid creating objects that won't be used
3. **Cache computed values**: Store repeated calculations in variables

.. code-block:: xml

    <!-- Good: compute once -->
    <variable name="SPACING" value="$f{ROOM_SIZE / NUM_ITEMS}"/>
    <for var="i" from="0" to="${NUM_ITEMS}">
        <block name="item_${i}">
            <init_pose>$f{i*SPACING} 0 0</init_pose>
        </block>
    </for>

Code organization
^^^^^^^^^^^^^^^^^^

1. **Use descriptive variable names**: ``NUM_OBSTACLES`` instead of ``N``
2. **Group related configurations**: Keep variables together at the top
3. **Comment complex logic**: Explain non-obvious conditions or expressions
4. **Extract to included files**: Move reusable patterns to separate files

.. code-block:: xml

    <!-- Main world file -->
    <world>
        <variable name="WAREHOUSE_MODE" value="true"/>
        
        <if condition="${WAREHOUSE_MODE}">
            <include file="warehouse_layout.xml"/>
        </if>
        
        <if condition="$f{not ${WAREHOUSE_MODE}}">
            <include file="open_field.xml"/>
        </if>
    </world>


Debugging
^^^^^^^^^^

Enable verbose parsing to see variable evaluations:

.. code-block:: bash

    MVSIM_VERBOSE_PARSE=1 mvsim myworld.world.xml

This will print each variable substitution and expression evaluation, helping you debug 
complex flow control logic.


Expression syntax reference
============================

Within ``$f{...}`` expressions, you can use:

Mathematical operators
^^^^^^^^^^^^^^^^^^^^^^^^

- Arithmetic: ``+``, ``-``, ``*``, ``/``, ``%``, ``^`` (power)
- Comparison: ``<``, ``>``, ``<=``, ``>=``, ``==``, ``!=``
- Logical: ``and``, ``or``, ``not``
- Bitwise: ``&``, ``|``, ``~``, ``<<``, ``>>``

Functions
^^^^^^^^^^

- **Trigonometric**: ``sin``, ``cos``, ``tan``, ``asin``, ``acos``, ``atan``, ``atan2``
- **Math**: ``sqrt``, ``abs``, ``exp``, ``log``, ``log10``, ``pow``, ``ceil``, ``floor``, ``round``
- **Random**: ``rand()`` (0 to 1), ``randn()`` (normal distribution)
- **String**: ``strcmp(s1, s2)`` (returns 0 if equal)
- **Conversion**: ``int(x)`` (convert to integer)

Constants
^^^^^^^^^^^

- ``M_PI``: π (3.14159...)
- ``M_E``: e (2.71828...)

See the `ExprTk documentation <http://www.partow.net/programming/exprtk/>`_ for the complete 
expression language reference.


Complete example
==================

Here's a complete world demonstrating advanced flow control:

.. code-block:: xml

    <?xml version="1.0"?>
    <world>
        <!-- Configuration -->
        <variable name="WORLD_SIZE" value="30"/>
        <variable name="NUM_ZONES" value="3"/>
        <variable name="DEBUG_MODE" value="false"/>
        
        <!-- Create zones with different obstacle patterns -->
        <for var="zone" from="0" to="$f{NUM_ZONES-1}">
            <variable name="zone_x" value="$f{zone * WORLD_SIZE}"/>
            
            <!-- Zone markers (only in debug mode) -->
            <if condition="${DEBUG_MODE}">
                <block name="zone_marker_${zone}">
                    <init_pose>${zone_x} 0 0</init_pose>
                    <geometry type="cylinder" radius="1.0" length="5.0"/>
                    <color>#00FF00</color>
                    <intangible>true</intangible>
                </block>
            </if>
            
            <!-- Different pattern per zone -->
            <for var="i" from="0" to="10">
                <!-- Zone 0: Regular grid -->
                <if condition="$f{zone == 0}">
                    <block name="grid_${zone}_${i}">
                        <init_pose>$f{zone_x + i*2} $f{i*2} 0</init_pose>
                        <geometry type="box" lx="1" ly="1" lz="1"/>
                    </block>
                </if>
                
                <!-- Zone 1: Circular arrangement -->
                <if condition="$f{zone == 1}">
                    <block name="circle_${zone}_${i}">
                        <init_pose>
                            $f{zone_x + 10 + 8*cos(i*2*M_PI/10)}
                            $f{10 + 8*sin(i*2*M_PI/10)}
                            0
                        </init_pose>
                        <geometry type="cylinder" radius="0.5" length="2"/>
                    </block>
                </if>
                
                <!-- Zone 2: Random placement -->
                <if condition="$f{zone == 2}">
                    <block name="random_${zone}_${i}">
                        <init_pose>
                            $f{zone_x + rand()*WORLD_SIZE}
                            $f{rand()*WORLD_SIZE}
                            $f{rand()*360}
                        </init_pose>
                        <geometry type="box" 
                                  lx="$f{0.5+rand()*1.5}" 
                                  ly="$f{0.5+rand()*1.5}" 
                                  lz="$f{1+rand()*2}"/>
                    </block>
                </if>
            </for>
        </for>
    </world>

This example creates three distinct zones, each with a different obstacle pattern, 
and includes debug markers that can be toggled on/off.