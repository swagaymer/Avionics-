library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.numeric_std.ALL; 

entity FlightControlPanel is
    Port (
        -- Inputs 
        clk: in std_logic;  -- Clock signal
        rst: in std_logic;  -- Reset signal
        enable_motor: in std_logic;  -- Signal to enable the main motor
        enable_motor_redundant: in std_logic;  -- Signal to enable the redundant motor
        control_signal: in std_logic_vector(2 downto 0);  -- Control signal for main motor
        control_signal_redundant: in std_logic_vector(2 downto 0);  -- Control signal for redundant motor
        aileron_angle: in integer range -90 to 90;  -- Angle of aileron deflection
        elevator_angle: in integer range -90 to 90;  -- Angle of elevator deflection
        rudder_angle: in integer range -90 to 90;  -- Angle of rudder deflection
        aircraft_speed: in integer range 0 to 500; -- Placeholder for aircraft speed
        wind_resistance: in integer range 0 to 100; -- Placeholder for wind resistance
        throttle_position: in integer range 0 to 100; -- Throttle position 0 to 100
		
        -- Outputs
        motor_output: out std_logic;  -- Output signal for main motor
        motor_output_redundant: out std_logic;  -- Output signal for redundant motor
        aileron_output: out std_logic;  -- Output signal for aileron control
        elevator_output: out std_logic;  -- Output signal for elevator control
        rudder_output: out std_logic;  -- Output signal for rudder control
        yoke_control: out integer range -90 to 90;  -- Yoke control angle
		
        -- ARINC 429 Data Bus
        tx_data: out std_logic_vector(31 downto 0);  -- Data to be transmitted on ARINC 429 data bus
        rx_data: in std_logic_vector(31 downto 0);  -- Received data from ARINC 429 data bus
        tx_enable: out std_logic;  -- Enable signal for ARINC 429 data bus transmission
        rx_enable: out std_logic;  -- Enable signal for ARINC 429 data bus reception
        -- RAM Signals
        ram_data: in std_logic_vector(31 downto 0);  -- Data read from RAM
        ram_addr: in std_logic_vector(7 downto 0);  -- Address for RAM read/write
        ram_write_enable: in std_logic := '0';  -- Write enable signal for RAM/0 def val
        ram_read_enable: in std_logic := '0'  -- Read enable signal for RAM
    );
end entity FlightControlPanel;

architecture Behavioral of FlightControlPanel is
    -- Declare signals and components here
    signal motor_output_internal : std_logic;
    signal motor_output_internal_redundant : std_logic;
    signal aileron_output_internal : std_logic;
    signal elevator_output_internal : std_logic;
    signal rudder_output_internal : std_logic;
    signal yoke_control_internal : integer range -90 to 90 := 0; -- Initial yoke control angle

    -- ARINC 429 Data Bus Signals
    signal tx_data_internal: std_logic_vector(31 downto 0) := (others => '0');
    signal tx_enable_internal: std_logic := '0';
    signal rx_enable_internal: std_logic := '0';
    
    -- RAM Signals
    signal ram_data_internal: std_logic_vector(31 downto 0) := (others => '0');
    
    -- Motor Components
    component Motor
        port (
            enable: in std_logic;
            control_signal: in std_logic_vector(2 downto 0);
            motor_output: out std_logic
        );
    end component Motor;

    -- Aileron Control Component
    component AileronControl
        port (
            aileron_angle: in integer range -90 to 90;
            aileron_output: out std_logic
        );
    end component AileronControl;

    -- Elevator Control Component
    component ElevatorControl
        port (
            elevator_angle: in integer range -90 to 90;
            elevator_output: out std_logic
        );
    end component ElevatorControl;

    -- Rudder Control Component
    component RudderControl
        port (
            rudder_angle: in integer range -90 to 90;
            rudder_output: out std_logic
        );
    end component RudderControl;

    -- Redundant Motor Control Component
    component MotorRedundant
        port (
            enable: in std_logic;
            control_signal: in std_logic_vector(2 downto 0);
            motor_output: out std_logic
        );
    end component MotorRedundant;

    -- ARINC 429 Data Bus Component
    component ARINC429DataBus
        port (
            clk: in std_logic;
            rst: in std_logic;
            enable_motor: in std_logic;
            tx_data: out std_logic_vector(31 downto 0);
            rx_data: in std_logic_vector(31 downto 0);
            tx_enable: out std_logic;
            rx_enable: out std_logic
        );
    end component ARINC429DataBus;

    -- RAM Component
    component RAM
        port (
            clk: in std_logic;
            rst: in std_logic;
            data: in std_logic_vector(31 downto 0);
            addr: in std_logic_vector(7 downto 0);
            write_enable: in std_logic;
            read_enable: in std_logic;
            output: out std_logic_vector(31 downto 0)
        );
    end component RAM;

begin
    -- Motor Instantiations
    MainMotor: Motor port map (
        enable => enable_motor,
        control_signal => control_signal,
        motor_output => motor_output_internal
    );

    RedundantMotor: MotorRedundant port map (
        enable => enable_motor_redundant,
        control_signal => control_signal_redundant,
        motor_output => motor_output_internal_redundant
    );

    -- Aileron Control Instantiation
    AileronControl_Instance: AileronControl port map (
        aileron_angle => aileron_angle,
        aileron_output => aileron_output_internal
    );

    -- Elevator Control Instantiation
    ElevatorControl_Instance: ElevatorControl port map (
        elevator_angle => elevator_angle,
        elevator_output => elevator_output_internal
    );

    -- Rudder Control Instantiation
    RudderControl_Instance: RudderControl port map (
        rudder_angle => rudder_angle,
        rudder_output => rudder_output_internal
    );

    -- ARINC 429 Data Bus Instantiation
    ARINC429DataBus_Instance: ARINC429DataBus port map (
        clk => clk,
        rst => rst,
        enable_motor => enable_motor,
        tx_data => tx_data_internal,
        rx_data => rx_data,
        tx_enable => tx_enable_internal,
        rx_enable => rx_enable_internal
    );

    -- RAM Instantiation
    RAM_Instance: RAM port map (
        clk => clk,
        rst => rst,
        data => ram_data,
        addr => ram_addr,
        write_enable => ram_write_enable,
        read_enable => ram_read_enable,
        output => ram_data_internal
    );

    -- Redundancy Voting Process
    process
    begin
         if enable_motor = '1' and enable_motor_redundant = '1' then
        if motor_output_internal = '1' and motor_output_internal_redundant = '1' then
            motor_output <= '1'; -- Both primary and redundant motors are active
        elsif motor_output_internal = '0' and motor_output_internal_redundant = '0' then
            motor_output <= '0'; -- Both primary and redundant motors are inactive
        elsif motor_output_internal = '0' and motor_output_internal_redundant = '1' then
            motor_output <= '1'; -- Primary motor is inactive, but redundant motor is active
        else
          motor_output <= '0'; -- Any other combination, default to turning off the motor
        end if;
    else
        motor_output <= '0'; -- Either the primary or redundant motor is not enabled
    end if;
    end process;

    -- ARINC 429 Data Bus Logic
    process (clk, rst)
    begin
        if rst = '1' then
            tx_enable_internal <= '0';
            rx_enable_internal <= '0';
        elsif rising_edge(clk) then
            tx_enable_internal <= enable_motor;
            tx_data_internal <= std_logic_vector(to_unsigned(yoke_control_internal, 32));
        end if;
    end process;

    -- RAM Process
    process (clk, rst)
    begin
        if rst = '1' then
            ram_data_internal <= (others => '0');
        elsif rising_edge(clk) then
            if ram_write_enable = '1' then
                ram_data_internal(to_integer(unsigned(ram_addr))) <= ram_data(31 downto 0);
            elsif ram_read_enable = '1' then
                ram_data_internal <= ram_data_internal;
            end if;
        end if;
    end process;

    -- Output assignments
    motor_output_redundant <= motor_output_internal_redundant;
    aileron_output <= aileron_output_internal;
    elevator_output <= elevator_output_internal;
    rudder_output <= rudder_output_internal;
    yoke_control <= yoke_control_internal;

    -- ARINC 429 Data Bus Output Assignments
    tx_data <= tx_data_internal;
    tx_enable <= tx_enable_internal;
    rx_enable <= rx_enable_internal;

end Behavioral;

