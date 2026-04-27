library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity conv_top is
    Port (
        clk         : in  std_logic;
        rst         : in  std_logic;
        kernel_sel  : in  std_logic_vector(1 downto 0);
        uart_rx_pin : in  std_logic;
        uart_tx_pin : out std_logic;
        led_out     : out std_logic_vector(7 downto 0)
    );
end conv_top;

architecture Behavioral of conv_top is

    constant IMG_W : integer := 256;

    component uart_rx
        Generic (CLK_FREQ : integer := 100_000_000; BAUD_RATE : integer := 115_200);
        Port (
            clk        : in  std_logic;
            rst        : in  std_logic;
            rx         : in  std_logic;
            data_out   : out std_logic_vector(7 downto 0);
            data_valid : out std_logic
        );
    end component;

    component uart_tx
        Generic (CLK_FREQ : integer := 100_000_000; BAUD_RATE : integer := 115_200);
        Port (
            clk      : in  std_logic;
            rst      : in  std_logic;
            data_in  : in  std_logic_vector(7 downto 0);
            tx_start : in  std_logic;
            tx       : out std_logic;
            tx_busy  : out std_logic
        );
    end component;

    type line_t is array (0 to IMG_W-1) of std_logic_vector(7 downto 0);
    signal lineA : line_t := (others => (others => '0'));
    signal lineB : line_t := (others => (others => '0'));
    signal lineC : line_t := (others => (others => '0'));

    signal top_sel  : integer range 0 to 2 := 0;
    signal mid_sel  : integer range 0 to 2 := 1;
    signal bot_sel  : integer range 0 to 2 := 2;

    signal out_buf : line_t := (others => (others => '0'));

    signal rx_data    : std_logic_vector(7 downto 0);
    signal rx_valid   : std_logic;
    signal tx_data    : std_logic_vector(7 downto 0) := (others => '0');
    signal tx_start   : std_logic := '0';
    signal tx_busy    : std_logic;

    -- TX bug fix: tx_busy pickup bekleme
    signal tx_wait_busy : std_logic := '0';

    signal col_count  : integer range 0 to IMG_W := 0;
    signal tx_idx     : integer range 0 to IMG_W := 0;
    signal conv_col   : integer range 0 to IMG_W-1 := 0;

    type state_t is (S_FILL_LINE0, S_FILL_LINE1, S_FILL_LINE2,
                     S_CONV, S_TX, S_NEXT_LINE);
    signal state : state_t := S_FILL_LINE0;

    type kernel_t is array (0 to 8) of integer range -16 to 16;
    signal kernel : kernel_t;
    signal divisor : integer range 1 to 16;

begin

    RX_INST : uart_rx
        port map (clk => clk, rst => rst, rx => uart_rx_pin,
                  data_out => rx_data, data_valid => rx_valid);

    TX_INST : uart_tx
        port map (clk => clk, rst => rst, data_in => tx_data,
                  tx_start => tx_start, tx => uart_tx_pin, tx_busy => tx_busy);

    KERNEL_SELECT : process(kernel_sel)
    begin
        case kernel_sel is
            when "00" =>
                kernel  <= (1, 2, 1,  2, 4, 2,  1, 2, 1);
                divisor <= 16;
            when "01" =>
                kernel  <= (0, -1, 0,  -1, 5, -1,  0, -1, 0);
                divisor <= 1;
            when "10" =>
                kernel  <= (-1, -1, -1,  -1, 8, -1,  -1, -1, -1);
                divisor <= 1;
            when others =>
                kernel  <= (-2, -1, 0,  -1, 1, 1,  0, 1, 2);
                divisor <= 1;
        end case;
    end process;

    MAIN_FSM : process(clk)
        variable sum : integer;
        variable result : integer;
        variable p00, p01, p02 : integer;
        variable p10, p11, p12 : integer;
        variable p20, p21, p22 : integer;
    begin
        if rising_edge(clk) then
            if rst = '1' then
                state        <= S_FILL_LINE0;
                col_count    <= 0;
                tx_idx       <= 0;
                tx_start     <= '0';
                tx_wait_busy <= '0';
                conv_col     <= 0;
                top_sel      <= 0;
                mid_sel      <= 1;
                bot_sel      <= 2;
            else
                tx_start <= '0';

                case state is
                    when S_FILL_LINE0 =>
                        if rx_valid = '1' then
                            lineA(col_count) <= rx_data;
                            if col_count = IMG_W-1 then
                                col_count <= 0;
                                state     <= S_FILL_LINE1;
                            else
                                col_count <= col_count + 1;
                            end if;
                        end if;

                    when S_FILL_LINE1 =>
                        if rx_valid = '1' then
                            lineB(col_count) <= rx_data;
                            if col_count = IMG_W-1 then
                                col_count <= 0;
                                state     <= S_FILL_LINE2;
                            else
                                col_count <= col_count + 1;
                            end if;
                        end if;

                    when S_FILL_LINE2 =>
                        if rx_valid = '1' then
                            case bot_sel is
                                when 0      => lineA(col_count) <= rx_data;
                                when 1      => lineB(col_count) <= rx_data;
                                when others => lineC(col_count) <= rx_data;
                            end case;
                            if col_count = IMG_W-1 then
                                col_count <= 0;
                                conv_col  <= 0;
                                state     <= S_CONV;
                            else
                                col_count <= col_count + 1;
                            end if;
                        end if;

                    when S_CONV =>
                        if conv_col = 0 then
                            case mid_sel is
                                when 0      => out_buf(0) <= lineA(0);
                                when 1      => out_buf(0) <= lineB(0);
                                when others => out_buf(0) <= lineC(0);
                            end case;
                            conv_col <= 1;

                        elsif conv_col = IMG_W-1 then
                            case mid_sel is
                                when 0      => out_buf(IMG_W-1) <= lineA(IMG_W-1);
                                when 1      => out_buf(IMG_W-1) <= lineB(IMG_W-1);
                                when others => out_buf(IMG_W-1) <= lineC(IMG_W-1);
                            end case;
                            conv_col <= 0;
                            tx_idx   <= 0;
                            state    <= S_TX;

                        else
                            case top_sel is
                                when 0 =>
                                    p00 := to_integer(unsigned(lineA(conv_col-1)));
                                    p01 := to_integer(unsigned(lineA(conv_col)));
                                    p02 := to_integer(unsigned(lineA(conv_col+1)));
                                when 1 =>
                                    p00 := to_integer(unsigned(lineB(conv_col-1)));
                                    p01 := to_integer(unsigned(lineB(conv_col)));
                                    p02 := to_integer(unsigned(lineB(conv_col+1)));
                                when others =>
                                    p00 := to_integer(unsigned(lineC(conv_col-1)));
                                    p01 := to_integer(unsigned(lineC(conv_col)));
                                    p02 := to_integer(unsigned(lineC(conv_col+1)));
                            end case;

                            case mid_sel is
                                when 0 =>
                                    p10 := to_integer(unsigned(lineA(conv_col-1)));
                                    p11 := to_integer(unsigned(lineA(conv_col)));
                                    p12 := to_integer(unsigned(lineA(conv_col+1)));
                                when 1 =>
                                    p10 := to_integer(unsigned(lineB(conv_col-1)));
                                    p11 := to_integer(unsigned(lineB(conv_col)));
                                    p12 := to_integer(unsigned(lineB(conv_col+1)));
                                when others =>
                                    p10 := to_integer(unsigned(lineC(conv_col-1)));
                                    p11 := to_integer(unsigned(lineC(conv_col)));
                                    p12 := to_integer(unsigned(lineC(conv_col+1)));
                            end case;

                            case bot_sel is
                                when 0 =>
                                    p20 := to_integer(unsigned(lineA(conv_col-1)));
                                    p21 := to_integer(unsigned(lineA(conv_col)));
                                    p22 := to_integer(unsigned(lineA(conv_col+1)));
                                when 1 =>
                                    p20 := to_integer(unsigned(lineB(conv_col-1)));
                                    p21 := to_integer(unsigned(lineB(conv_col)));
                                    p22 := to_integer(unsigned(lineB(conv_col+1)));
                                when others =>
                                    p20 := to_integer(unsigned(lineC(conv_col-1)));
                                    p21 := to_integer(unsigned(lineC(conv_col)));
                                    p22 := to_integer(unsigned(lineC(conv_col+1)));
                            end case;

                            sum := p00*kernel(0) + p01*kernel(1) + p02*kernel(2) +
                                   p10*kernel(3) + p11*kernel(4) + p12*kernel(5) +
                                   p20*kernel(6) + p21*kernel(7) + p22*kernel(8);

                            result := sum / divisor;

                            if result < 0 then
                                result := 0;
                            elsif result > 255 then
                                result := 255;
                            end if;

                            out_buf(conv_col) <= std_logic_vector(to_unsigned(result, 8));
                            conv_col <= conv_col + 1;
                        end if;

                    -- BUG FIX: tx_busy pickup bekleme
                    when S_TX =>
                        if tx_wait_busy = '0' then
                            -- Yeni byte gonder
                            if tx_busy = '0' then
                                tx_data      <= out_buf(tx_idx);
                                tx_start     <= '1';
                                tx_wait_busy <= '1';
                            end if;
                        else
                            -- Bir onceki gonderimin bitmesini bekle
                            if tx_busy = '0' then
                                tx_wait_busy <= '0';
                                if tx_idx = IMG_W-1 then
                                    tx_idx <= 0;
                                    state  <= S_NEXT_LINE;
                                else
                                    tx_idx <= tx_idx + 1;
                                end if;
                            end if;
                        end if;

                    when S_NEXT_LINE =>
                        top_sel   <= mid_sel;
                        mid_sel   <= bot_sel;
                        bot_sel   <= top_sel;
                        col_count <= 0;
                        state     <= S_FILL_LINE2;

                end case;
            end if;
        end if;
    end process;

    led_out(0) <= '1' when state = S_FILL_LINE0 else '0';
    led_out(1) <= '1' when state = S_FILL_LINE1 else '0';
    led_out(2) <= '1' when state = S_FILL_LINE2 else '0';
    led_out(3) <= '1' when state = S_CONV        else '0';
    led_out(4) <= '1' when state = S_TX          else '0';
    led_out(5) <= kernel_sel(0);
    led_out(6) <= kernel_sel(1);
    led_out(7) <= '1';

end Behavioral;
