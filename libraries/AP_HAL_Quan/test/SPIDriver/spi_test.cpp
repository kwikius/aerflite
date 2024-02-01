
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_HAL/utility/functor.h>
#include <AP_Math/vector3.h>
#include <cmath>
#include <quantracker/osd/osd.hpp>
#include <task.h>
#include <cstring>
#include <stm32f4xx.h>

#error "This isnt connected"
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#error the quan and quan aerflite boards don't expose the spi driver
#endif
#endif

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

namespace {

   constexpr uint8_t red_led_pin = 1U;
   constexpr uint8_t pin_off = 0U;
   constexpr uint8_t pin_on = 1U;

}

/*
   Want an intr pin
   DMA to read the data
   IRQ on dma done 
   put data on a freertos queue wit timestamp
   eg.
   main reads data that was acquired then swap buffs
   the task 
*/

namespace{
     
      struct mpu{
          // TODO  add mag
         mpu( AP_HAL::SPIDeviceDriver* device_in)
         :m_gyro{*this}
         ,m_accel{*this}
         ,m_device{device_in}
         ,m_good{false}
         ,m_sample_frequency_Hz{50}{}

         // some useful register values
         struct val{
            static constexpr uint8_t device_wakeup = 0U;
            static constexpr uint8_t device_reset = (1 << 7);
            static constexpr uint8_t i2c_if_dis = (1U << 4U);
            static constexpr uint8_t whoami = 0x71;
         };
      
         // some useful imu register addresses
         struct reg{
            static constexpr uint8_t sample_rate_div     = 25U;
            static constexpr uint8_t config              = 26U;
            static constexpr uint8_t gyro_config         = 27U;
            static constexpr uint8_t accel_config        = 27U;
            static constexpr uint8_t accel_config2       = 28U;
            static constexpr uint8_t fifo_enable         = 35U;
            static constexpr uint8_t intr_bypass_en_cfg  = 55U;
            static constexpr uint8_t intr_enable         = 56U;
            static constexpr uint8_t intr_status         = 58U;
            static constexpr uint8_t accel_measurements  = 59U; // 59 to 64
            static constexpr uint8_t temp_measurements   = 65U; // 65 to 66
            static constexpr uint8_t gyro_measurements   = 67U; // 67 to 72
            static constexpr uint8_t signal_path_reset   = 104U; 
            static constexpr uint8_t accel_intr_ctrl     = 105U; 
            // user ctrl bit 7 enable DMP
            // user ctrl bit 3 reset DMP
            static constexpr uint8_t user_ctrl           = 106U;
            static constexpr uint8_t pwr_mgmt1           = 107U;
            static constexpr uint8_t pwr_mgmt2           = 108U;
            // DMP specific regs here
            static constexpr uint8_t fifo_count          = 114U; // 114 to 115
            static constexpr uint8_t fifo_read_write     = 116U; // 59 to 64
            static constexpr uint8_t whoami              = 117U; 
            static constexpr uint8_t accel_offsets       = 119U; // 119 to 126
         };

//#######################################################################################################

//#######################################################################################################

//#######################################################################################################
         bool init()
         {
            if (m_device == nullptr){
               m_good = false;
               return false;
            }
            auto * const sem = m_device->get_semaphore();
            if ( sem && sem->take_nonblocking() ){
                disable_i2c_mode();
                reset();
                if (! check_whoami() ){
                  return false;
                }
            }else{
               hal.console->printf("(init) couldnt get spi semaphore\n");
               return false;
            }
            wakeup();
            select_clk(); 
            setup_filters(); 
            setup_fifo();
            setup_interrupts();

            set_high_speed();
            sem->give();
            m_good = true;
            return m_good;
         }

//#######################################################################################################

//#######################################################################################################

//#######################################################################################################
      private:

         // output gyro and accel at 1 kHz
         // see AP_InertialSensor_MPU6000.h
         struct gyro_t {

            gyro_t(mpu & m) : m_mpu(m){}

            void setup_filter()
            {
               // reg::gyro_config, fchoice_b == 0b00;
               // reg::config , DLPF_CFG = 1  this applies to gyro and temperature
               // with reg::sample_rate_div == 0, gyro data is output at 1 kHz 
                // LPF can be bandwidth 184 to 5Hz by changing filter_code between 1 and 6
                constexpr uint8_t filter_code = 1;
                uint8_t const v = m_mpu.reg_read(reg::config) & ~(0b111);
                m_mpu.reg_write(reg::config,v & filter_code);

                uint8_t const v1 = m_mpu.reg_read(reg::gyro_config) & ~(0b11);
                m_mpu.reg_write(reg::gyro_config,v1 );
            }
   
            bool set_full_scale_range_deg_per_sec(int32_t range_deg_per_sec)
            {
               int32_t const code = log2(std::abs(range_deg_per_sec) / 250);
               if ((code >= 0) && ( code < 4)){
                  uint8_t v = m_mpu.reg_read(reg::gyro_config) & ~(0b11 << 3U);
                  m_mpu.reg_write(reg::gyro_config, v | ( static_cast<uint32_t>(code) << 3U));
                  return true;
               }
               return false;
            }

            uint16_t get_full_scale_range_deg_per_sec()const
            {
               uint8_t const code = (m_mpu.reg_read(reg::gyro_config) & (0b11 << 3)) >> 3;
               return  (1U << code) * 250U;
            }
  
            mpu & m_mpu;
         } m_gyro;

         // accel scale is set to 8g
         struct accel_t{
            accel_t(mpu& m): m_mpu(m){}
            void setup_filter()
            {
              // want a 1 kHz output when sample_rate_div == 0;
              // and reg::accel_config2 , accel_fchoice_b == 0
              // reg::accel_config2,A_DLP_CFG =  between 0 and 6 inc

               // filter code can be from 0 to 6
               // higher values give a lower cutoff frequency
               // bandwidth 460 Hz to 5Hz
               // then data rate = 1 kHz
               constexpr uint8_t filter_code = 1;
               uint8_t const v = m_mpu.reg_read(reg::accel_config2) &  ~(0b1111);
               m_mpu.reg_write(reg::accel_config2, v | filter_code);
            }
            bool set_full_scale_range_g(int16_t value)
            {
               // ±2g (00), ±4g (01), ±8g (10), ±16g (11)
               int16_t const code = log2(std::abs(value))-1;
               if ( (code >= 0) && (code < 4)){
                  uint8_t const v = m_mpu.reg_read(reg::accel_config) & ~(0b11 << 3); 
                  m_mpu.reg_write(reg::accel_config,v | (code << 3));
                  return true;
               }else{
                  return false;
               }
            }
            uint16_t get_full_scale_range_g()const
            {
               uint8_t const v = (m_mpu.reg_read(reg::accel_config) & (0b11 << 3)) >> 3;
               return 1 << (v + 1);
            }

          
         private:
            mpu& m_mpu;
           
         } m_accel;
public:
          bool have_data_ready_interrupt()const
            {
               return reg_getbit<0>(reg::intr_status);
            }

            void clear_data_ready_interrupt()
            {
               reg_clearbit<0>(reg::intr_status);
            }

          bool check_whoami()
         {
            hal.console->printf("Checking device is a MPU9250\n");
            uint8_t const whoami = reg_read(reg::whoami);
            if ( whoami == val::whoami){
                hal.console->printf("Whoa Am I returned %u\n", static_cast<unsigned int>(whoami));

                hal.console->printf("%u%u%u%u,%u%u%u%u\n"
                  ,static_cast<unsigned int>(reg_getbit<7>(reg::whoami))
                  ,static_cast<unsigned int>(reg_getbit<6>(reg::whoami))
                  ,static_cast<unsigned int>(reg_getbit<5>(reg::whoami))
                  ,static_cast<unsigned int>(reg_getbit<4>(reg::whoami))
                  ,static_cast<unsigned int>(reg_getbit<3>(reg::whoami))
                  ,static_cast<unsigned int>(reg_getbit<2>(reg::whoami))
                  ,static_cast<unsigned int>(reg_getbit<1>(reg::whoami))
                  ,static_cast<unsigned int>(reg_getbit<0>(reg::whoami))
               );

               return true;
            }else{
               hal.console->printf("who am I returned %u",static_cast<unsigned int>(whoami));
               return false;
            }
         }

         template <uint8_t N>
         void read_fifo(uint8_t* buf_out)
         {
             uint8_t tx[N + 1];
             uint8_t rx[N + 1];
             tx[0] = reg::fifo_read_write;
             memset(tx +1, 0, N);
             m_device->transaction(tx,rx, N);
             memcpy( buf_out,rx+1, N);
              
         }
private:
         // MPU9250 starts up in sleep mode
         // Accelerometer Takes 20 ms to come out of sleep
         // Acc Takes 30 ms from cold start
         // Gyro takes 35 ms from sleep
         void wakeup()
         {
            hal.console->printf("Waking up device\n");
            reg_write(reg::pwr_mgmt1 ,0);
            hal.scheduler->delay(50);
         }

         void reset()
         {
            hal.console->printf("Resetting device\n");
            reg_setbit<7>(reg::pwr_mgmt1 );
            hal.scheduler->delay(100);
         }

         void disable_i2c_mode()
         {
             reg_setbit<4>(reg::user_ctrl);
             hal.scheduler->delay(10);
         }

         void disable_gyros_and_accels()
         {
            reg_write(reg::pwr_mgmt2, 0b00111111);
         }
//
//         void enable_gyros_and_accels()
//         {
//            reg_write(reg::pwr_mgmt2, 0);
//         }

         void select_clk()
         {
            reg_setbit<0>(reg::pwr_mgmt1);
            hal.scheduler->delay(50);
         }
         
        
         void setup_filters()
         {
            reg_write(reg::config, 0x3);         // DLP_CFG  = 3
            reg_write(reg::gyro_config,0x00);    // FCHOICE == 0b11
            reg_write(reg::accel_config2,0x01);  //ACCEL_FCHOICE == 0b1, A_DLP_CFG  == 1
            reg_write(reg::sample_rate_div,100); // should give 10 Hz?
            setup_gains();
         }

         void setup_gains()
         {
            m_gyro.set_full_scale_range_deg_per_sec(2000);
            m_accel.set_full_scale_range_g(8);
         }

         void setup_fifo()
         {

           // ideally want to accumulate
           // a set of gyro and accel vals
           // to read and then get mpu to set the int line
           // want accel and gyro but have to have temp also
           // so explicitly  set all here to save confusion
            reg_setbit<2>(reg::user_ctrl); // reset fifo
            reg_setbit<0>(reg::user_ctrl); // reset signal paths
            uint8_t const v = reg_read(reg::fifo_enable);
            reg_write(reg::fifo_enable, v | 0b1111100);
            reg_setbit<6>(reg::user_ctrl); // enable fifo
            
         }

         void set_high_speed()
         {
            m_device->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);
         }

         void setup_interrupts()
         {
//            uint8_t v = reg_read( reg::intr_bypass_en_cfg);
//            v = ( v  | (
//                 ( 1 << 7)  // set active lo
//               | ( 1 << 5)  // extintr pin is asserted till intr status is cleared
//            //   | ( 1 << 4)  // read of any reg clears
//               ) 
//            ) & ~(
//               (1 << 6)   // push pull not open drain
//            );
            taskENTER_CRITICAL();
            reg_write(reg::intr_bypass_en_cfg, ( 1 << 5));
            uint8_t const res = reg_read(reg::intr_bypass_en_cfg);
            taskEXIT_CRITICAL();
            if (  res != ( 1 << 5) ){
               hal.console->printf("failed to set reg\n");
            }else{
                hal.console->printf("read test ok\n");
            }
            
//            
//            // raw_rdy_en bit : assert interrupt pin when raw sensor data is ready
//           // reg_setbit<0>(reg::intr_enable);
//            reg_clearbit<0>(reg::intr_enable); // for now
            //clear flags
           // reg_write(reg::intr_status,0); 
         }

        
         void reg_write( uint8_t r, uint8_t v)
         {
            // taskENTER_CRITICAL();
             uint8_t arr[2] = {r, v};
             m_device->transaction( arr, nullptr, 2);
            // taskEXIT_CRITICAL();
         }

         uint8_t reg_read( uint8_t r)const
         {
           // taskENTER_CRITICAL();
            uint8_t arr_out[2] = {static_cast<uint8_t>(r | (1 << 7)),0};
            uint8_t arr_in [2] = {0,0};
            m_device->transaction(arr_out,arr_in, 2);
           // taskEXIT_CRITICAL();
            return arr_in[1];
         }

         template <uint8_t B>
         bool reg_getbit(uint8_t r)const
         {
            return (reg_read(r) & ( 1 << B) ) != 0U;
         }
         
         template <uint8_t B>
         void reg_setbit(uint8_t r)
         {
             reg_write(r ,reg_read(r) | (1 << B) );
         }

         template <uint8_t B>
         void reg_clearbit(uint8_t r)
         {
             reg_write(r ,reg_read(r) & ~(1 << B) );
         }

         void alt_start()
         {
            reg_write(reg::pwr_mgmt1,0x00);
            hal.scheduler->delay(100);
            reg_write(reg::pwr_mgmt1,0x01);
            reg_write(reg::pwr_mgmt2, 0x0);
            reg_write(reg::config,0x03);
            reg_write(reg::gyro_config,0);
            reg_write(reg::sample_rate_div,0x04); 
            reg_write(reg::accel_config,2 << 3);
            reg_write(reg::accel_config2,0x03);
         }
public:
         void read_accel_data( uint8_t* buf)
         {

            for (uint8_t i = 0;i < 6; ++i){
               buf[i] = reg_read(reg::accel_measurements + i);
            }
         }
private:
         AP_HAL::SPIDeviceDriver* m_device;
         bool m_good;
         uint32_t m_sample_frequency_Hz;
      };

      mpu* pmpu = nullptr;
}

// called once after init of hal before startup of apm task
void setup() 
{
   hal.gpio->pinMode(red_led_pin,HAL_GPIO_OUTPUT);
   hal.gpio->write(red_led_pin,pin_off);
   pmpu = new mpu{ hal.spi->device(AP_HAL::SPIDevice_MPU9250) };
   pmpu->init();
}

void quan::uav::osd::on_draw() 
{ 
   draw_text("Quan APM SPI test",{-140,50});
}

namespace {
   constexpr uint8_t interval = 100U;
   uint64_t next_led_event = interval;
   uint64_t next_print_event = 333U;
    int count = 0;
}
// called forever in apm_task
void loop() 
{
   uint32_t const now = AP_HAL::millis();
   if ( next_led_event <= now ){
      hal.gpio->toggle(red_led_pin);
      next_led_event = now + interval;
   }
  
   //if ( next_print_event <= now){
      if(pmpu->have_data_ready_interrupt()){
         pmpu->clear_data_ready_interrupt();
         uint8_t buf[6];
         memset(buf,0,6);
         pmpu->read_accel_data(buf);
         Vector3<float> accel;
         union{
            uint8_t ar[2];
            int16_t val;
         }u;
         u.ar[0] = buf[1];
         u.ar[1] = buf[0];
         accel.x = u.val;
         u.ar[0] = buf[3];
         u.ar[1] = buf[2];
         accel.y = u.val;
         u.ar[0] = buf[5];
         u.ar[1] = buf[4];
         accel.z = u.val;
         if ( ++ count == 100){
            count = 0;
            hal.console->printf("received at %u\n", static_cast<unsigned int>(now));
            hal.console->printf("accel = [%3.3f,%3.3f,%3.3f]\n",
               static_cast<double>(accel.x),
               static_cast<double>(accel.y),
               static_cast<double>(accel.z)
            );
         }
   }

}

#if defined QUAN_WITH_OSD_OVERLAY
AP_HAL_MAIN();
#else
void create_apm_task();
void create_timer_task();

extern "C" {
   int main (void) 
   {
      osd_setup(); 
      create_draw_task(); 
      create_apm_task(); 
      vTaskStartScheduler (); 
   }
}

namespace { 
   char dummy_param = 0; 
   TaskHandle_t task_handle = NULL; 
   void apm_task(void * params) 
   { 
      hal.init(0, NULL);
      setup();
      hal.scheduler->system_initialized(); 
      for(;;){ 
         loop(); 
      } 
   } 
} 

void create_apm_task() 
{ 
  xTaskCreate( 
      apm_task,"apm task", 
      5000, 
      &dummy_param, 
      tskIDLE_PRIORITY + 1, 
      &task_handle 
  ); 
}
#endif

