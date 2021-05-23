template_start = """
<sdf version="1.4">
  <world name="default">

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
"""

template_end = """
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <precon_iters>0</precon_iters>
          <sor>1.400000</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>
          <contact_surface_layer>0.01000</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
  </world>

</sdf>
"""

world = template_start

for i in range(3):
  y_offset = i*2 
  lane_name = f"lane_{i}"
  lane = f"""
    <!-- Bowling ball -->
    <include>
      <pose> 1 {y_offset} 0 0 0 0 </pose>
      <uri>model://bowling_ball</uri>
      <name>{lane_name}_ball</name>
    </include>

    <!-- Wall -->
    <include>
      <pose> 0 {y_offset + 0.7} 0 0 0 0 </pose>
      <uri>model://alley_wall</uri>
      <name>{lane_name}_right_wall</name>
    </include>

    <include>
      <pose> 0 {y_offset-0.7} 0 0 0 0 </pose>
      <uri>model://alley_wall</uri>
      <name>{lane_name}_left_wall</name>
    </include>


    <!-- Bowling pins -->
    <!-- First row -->
    <include>
      <pose> 3 {y_offset} 0 0 0 0 </pose>
      <uri>model://bowling_pin</uri>
      <name>{lane_name}_pin1</name>
    </include>

    <!-- Second row -->
    <include>
      <pose> 3.1732 {y_offset + -0.1} 0 0 0 0 </pose>
      <uri>model://bowling_pin</uri>
      <name>{lane_name}_pin2</name>
    </include>

    <include>
      <pose> 3.1732 {y_offset + 0.1} 0 0 0 0 </pose>
      <uri>model://bowling_pin</uri>
      <name>{lane_name}_pin3</name>
    </include>

    <!-- Third row -->
    <include>
      <pose> 3.346 {y_offset + -0.2} 0 0 0 0 </pose>
      <uri>model://bowling_pin</uri>
      <name>{lane_name}_pin4</name>
    </include>

    <include>
      <pose> 3.346 {y_offset} 0 0 0 0 </pose>
      <uri>model://bowling_pin</uri>
      <name>{lane_name}_pin5</name>
    </include>
  
    <include>
      <pose>3.346 {y_offset + 0.2} 0 0 0 0 </pose>
      <uri>model://bowling_pin</uri>
      <name>{lane_name}_pin6</name>
    </include>

    <!-- Fourth row -->
    <include>
      <pose>3.520 {y_offset + -0.3} 0 0 0 0 </pose>
      <uri>model://bowling_pin</uri>
      <name>{lane_name}_pin7</name>
    </include>
    
    <include>
      <pose>3.520 {y_offset + -0.1} 0 0 0 0 </pose>
      <uri>model://bowling_pin</uri>
      <name>{lane_name}_pin8</name>
    </include>

    <include>
      <pose>3.520 {y_offset + 0.1} 0 0 0 0 </pose>
      <uri>model://bowling_pin</uri>
      <name>{lane_name}_pin9</name>
    </include>

    <include>
      <pose>3.520 {y_offset + 0.3} 0 0 0 0 </pose>
      <uri>model://bowling_pin</uri>
      <name>{lane_name}_pin10</name>
    </include> """
  world += lane

world += template_end

print(world)
