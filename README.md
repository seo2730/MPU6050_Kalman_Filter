# MPU6050_Kalman_Filter
Using MPU6050 and check roll,pitch in MATLAB<br>
MPU6050 Source Code : https://github.com/jarzebski/Arduino-MPU6050<br>

# Arduino(itself) Kalman Filter
not completed

# Arduino(with MATLAB) Kalman Filter
Reference : https://github.com/seo2730/Filter_algorithm_by_MATLAB/tree/master/4.%20Kalman_Filter/6.%20Horizontal%20position<br>
In MPU6050_MATLAB directory, there are arduino and matlab files to communicate each other.<br>
MPU6050_MATLAB 파일에는 서로 통신을 위한 아두이노 파일과 매트랩 파일들이 있다.<br>
<br>
#### MATLAB Serial Communication
- To connect 

      s = serialport("COM13",115200,"Timeout",60);

- Close
          
      clear s
     
- Read data(String)

      data = readline(s);
      
- Split string data

      split_data = strsplit(data,', ');
      
- Get acceleration data

      a = str2double(split_data(1,1:3));
      
- Get gyro data
 
      p = str2double(split_data(1,4));
      q = str2double(split_data(1,5));
      r = str2double(split_data(1,6));
      
- Results changing covariance(Up : Roll, below : Pitch)
R : 10<br>

![KakaoTalk_20200703_165557049](https://user-images.githubusercontent.com/42115807/86472187-bc88dd00-bd79-11ea-80e8-7e417bbd978d.png)
![KakaoTalk_20200703_165556973](https://user-images.githubusercontent.com/42115807/86472193-beeb3700-bd79-11ea-8e11-eee97364e925.png)<br>
<br>

R : 0.0001<br>

![KakaoTalk_20200703_171455541](https://user-images.githubusercontent.com/42115807/86472426-1d181a00-bd7a-11ea-877a-2ff7b3e7eee8.png)
![KakaoTalk_20200703_171455617](https://user-images.githubusercontent.com/42115807/86472431-1ee1dd80-bd7a-11ea-97ff-c397147313c6.png)<br>
<br>

R : 0.01<br>

![KakaoTalk_20200703_182045748](https://user-images.githubusercontent.com/42115807/86472507-39b45200-bd7a-11ea-87cf-8679349e8158.png)
![KakaoTalk_20200703_182045849](https://user-images.githubusercontent.com/42115807/86472514-3c16ac00-bd7a-11ea-9ec9-2d7fbf5060f5.png)<br>
