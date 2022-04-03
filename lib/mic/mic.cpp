/**\mainpage
 *
 * File		mic.cpp
 * @date	09 Nov 2018
 * @version	1.0.0
 *
 */

/*! @file mic.c
 @brief Functions for managing Noise level measurements */
#include "mic.h"

/**********************************************************************************************************************/
/* constants */
/**********************************************************************************************************************/
const i2s_port_t I2S_PORT = I2S_NUM_0;
const int BLOCK_SIZE = 1024;

/* Measured values */
// int peak_to_peak_val;
int max_val;

int32_t readingFFT[256];
float readingDB;

const double RMS_HANN = 0.61177;
const uint8_t FULL_SCALE_DBSPL = 120;
const uint8_t BIT_LENGTH = 24;
const double FULL_SCALE_DBFS = 20*log10(pow(2,(BIT_LENGTH)));

// Read multiple samples at once and calculate the sound pressure
int32_t samples0[2*BLOCK_SIZE] = {0};

/*!
 * @brief This API is used to initialize I2S communication.
 *
 * @param[in] None
 *
 * @return Nothing
 */
void init_mic(void)
{
    /* I2S driver initialization */
    Serial.println("Configuring I2S...");
    esp_err_t err;

    // The I2S config as per the example
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
        .sample_rate = 16000,                         // 16KHz
        .bits_per_sample = I2S_BITS_PER_SAMPLE_24BIT, // could only get it to work with 32bits
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // although the SEL config should be left, it seems to transmit on right
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
        .dma_buf_count = 8,                           // number of buffers
        .dma_buf_len = BLOCK_SIZE                     // samples per buffer
    };

    // The pin config as per the setup
    const i2s_pin_config_t pin_config = {
        .bck_io_num = 14,   // BCKL - SCK
        .ws_io_num = 15,    // LRCL - WS
        .data_out_num = -1, // not used (only for speakers)
        .data_in_num = 32   // DOUT - SD
    };

    // Configuring the I2S driver and pins.
    // This function must be called before any I2S driver read/write operations.
    err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Failed installing driver: %d\n", err);
        while (true);
    }
    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("Failed setting pin: %d\n", err);
        while (true);
    }
    Serial.println("I2S driver installed OK.");
    /* I2S driver initialization complete */
}

// /*!
//  * @brief This API is used to get peak to peak value read from 128 samples.
//  *
//  * @param[in] None
//  *
//  * @return Nothing
//  */
// void get_peak_to_peak(void)
// {
//     // Read multiple samples at once and calculate the sound pressure
//     int32_t samples[BLOCK_SIZE] = {0};
//     int num_bytes_read = i2s_read_bytes(I2S_PORT, 
//                                         (char *)samples, 
//                                         BLOCK_SIZE,     // the doc says bytes, but its elements.
//                                         portMAX_DELAY); // no timeout
    
//     // Serial.print("Samples read: ");
//     // Serial.println(num_bytes_read);

//     int samples_read = num_bytes_read / 4;
//     if (num_bytes_read == BLOCK_SIZE) 
//     {
//         // compute the mean
//         float mean = 0;
//         // shift values so they are all positive
//         for (int i = 0; i < samples_read; ++i) 
//         {
//             samples[i] >>= 8;
//             samples[i] += pow(2,23);
//             mean += samples[i];
//         }
//         //Serial.println(mean/samples_read);

//         // variables for the 'peak to peak' max
//         int maxsample, minsample;
//         minsample = 100000;
//         maxsample = -100000;
//         for (int i=0; i<samples_read; i++) 
//         {
//             if (minsample > samples[i])
//             {
//                 minsample = samples[i];
//             }
            
//             if (maxsample < samples[i])
//             {
//                 maxsample = samples[i];
//             }
//         }

//         peak_to_peak_val = maxsample - minsample;

//         //------- cel mai probabil media si aplicata formula
//         //------- de la pag 1 din MICROPHONE SPECIFICATIONS EXPLAINED
//         // Serial.print("max: ");
//         // Serial.println(maxsample);

//         // Serial.print("min: ");
//         // Serial.println(minsample);

//         // Serial.print("'peak to peak' max: ");
//         //Serial.println(peak_to_peak_val);
//         Serial.println(compute_db_FS(peak_to_peak_val));
//     }
// }

/*!
 * @brief This API is used to get the max value read from 128 samples.
 *
 * @param[in] None
 *
 * @return Nothing
 */
void get_max(void)
{
    // Read multiple samples at once and calculate the sound pressure
    int32_t samples[BLOCK_SIZE] = {0};
    int num_bytes_read = i2s_read_bytes(I2S_PORT, (char *)samples, BLOCK_SIZE,     // the doc says bytes, but its elements.
	                             portMAX_DELAY); // no timeout
    
    // Serial.print("Samples read: ");
    // Serial.println(num_bytes_read);

    int samples_read = num_bytes_read / 4;
    if (num_bytes_read == BLOCK_SIZE) 
    {
        // make all values positive
        for (int i=0; i<samples_read; ++i) 
        {
            // shift values so they are all positive
            samples[i] >>= 8;
            //samples[i] += pow(2,23);
        }

        //compute mean and subtract it from each sample
        double meanval = 0;
        for (int i=0; i<samples_read; ++i) 
        {
            meanval += samples[i];
        }
        meanval /= samples_read;
        
        // subtract mean from all sapmles to get a 'normalized' output
        for (int i=0; i<samples_read; i++) 
        {
            samples[i] -= meanval;
            //Serial.println(samples[i]);
        }
        
        // variables for max
        int maxsample = 0;
        for (int i=0; i<samples_read; ++i) 
        {
            if (maxsample < samples[i])
            {
                maxsample = samples[i];
            }
        }

        max_val = maxsample;

        //Serial.println(maxsample);
        //Serial.println(compute_db_FS(maxsample));
    }
}

// /**
//  * @brief Get the peak_to_peak_val object
//  * 
//  *  @return int peak to peak value
//  */
// int get_peak_to_peak_val(void)
// {
//     return peak_to_peak_val;
// }

/**
 * @brief Get the max_val object
 * 
 *  @return int max value
 */
int get_max_val(void)
{
    return max_val;
}

// /**
//  * @brief compute db FS
//  *
//  *  @param[in] Raw value read by mic
//  * 
//  *  @return int db FS value
//  */
// int compute_db_FS(int value)
// {
//     double computed_value = 0;

//     // Formula is: sensitivity_sbFS = 20 x log_10(sensitivity_%FS / output_ref)
//     // output_ref = 1.0

//     computed_value = (value * 120) / (pow(2,24)-1);
//     //Serial.println(computed_value);
//     computed_value = round(computed_value);
//     //Serial.println(computed_value);

//     return int(computed_value);
// }

/* Smart citizen kit implementation */
/* This piece of code is modified from the repository 
   https://github.com/fablabbcn/smartcitizen-kit-20
   This is protected by the GPL License V3.0
   */
void getReading(void)
{
	// Read multiple samples at once and calculate the sound pressure
	int32_t samples[BLOCK_SIZE] = {0};

	// variables used for reading
	size_t num_bytes_read = 0;
	esp_err_t op_success = ESP_FAIL;

	//read 1024 bytes
	op_success = i2s_read(I2S_PORT
	                     , (char *)samples
						 , BLOCK_SIZE
						 , &num_bytes_read
						 , portMAX_DELAY);

	// compute number of read samples
    int samples_read = num_bytes_read / 4;

	// if read operation is sucessfull, perform analysis
    if ((num_bytes_read == BLOCK_SIZE) && (op_success == ESP_OK))
    {
        // process raw data
        for (int i=0; i<samples_read; ++i) 
        {
            // remove unused bits
            samples[i] >>= 7;
        }

		// add data to samples0 array
        for (int i=0; i<samples_read; ++i) 
        {
            samples0[i] = samples[i];
			samples[i]=0;
        }
	}

	// if read operation is sucessfull, perform analysis
    if ((num_bytes_read == BLOCK_SIZE) && (op_success == ESP_OK))
    {
		op_success = ESP_FAIL;

		op_success = i2s_read(I2S_PORT
							, (char *)samples
							, BLOCK_SIZE
							, &num_bytes_read
							, portMAX_DELAY);	
	}

	// if read operation is sucessfull, perform analysis
    if ((num_bytes_read == BLOCK_SIZE) && (op_success == ESP_OK))
    {
        // process raw data
        for (int i=0; i<samples_read; ++i) 
        {
            // remove unused bits
            samples[i] >>= 7;
        }

        // add data to samples0 array
        for (int i=0; i<samples_read; ++i) 
        {
            samples0[256+i] = samples[i];
			samples[i]=0;
        }

		// total samples read
		samples_read *= 2;

		//compute mean 
        int32_t sum = 0;
		for (uint16_t i=0; i<samples_read; i++) 
		{
			sum += samples0[i];
		}
		int32_t avg = sum / samples_read;
		
        // Center samples in zero
	    for (uint16_t i=0; i<samples_read; i++) 
        {
            samples0[i] = samples0[i] - avg;
        }

        // FFT
	    FFT(samples0, samples_read);

		// Equalization and A weighting
		for (uint16_t i=0; i<samples_read/2; i++)
		{
			readingFFT[i] *= (double)(equalWeight_A[i] / 65536.0);
		}

		// RMS
		long long rmsSum = 0;
		double rmsOut = 0;
		for (uint16_t i=0; i<samples_read/2; i++)
		{
			rmsSum += pow(readingFFT[i], 2) / (samples_read / 2);
		}
		rmsOut = sqrt(rmsSum);
		rmsOut = rmsOut * 1 / RMS_HANN * sqrt(samples_read/2) / sqrt(2);

		// Convert to dB
		readingDB = (double) (FULL_SCALE_DBSPL - (FULL_SCALE_DBFS - (20 * log10(rmsOut * sqrt(2)))));
		// Print value
		//Serial.println(readingDB);

		// save into max_val the integer value multiplied by 10
		max_val = round(readingDB*10);

		/*float maxsample = -1e8, minsample = 1e8;
		for (int i = 0; i < samples_read; ++i) 
		{
			minsample = min(minsample, samples0[i] - avg);
			maxsample = max(maxsample, samples0[i] - avg);
		}
		Serial.println(maxsample - minsample);

		// save into max_val the integer value showing the biggest difference
		max_val = maxsample - minsample;*/
	}
}
/* Modified version 
   Added the sencond parameter, samples_read
*/
bool FFT(int32_t *source, int samples_read)
{
	int16_t scaledSource[samples_read]={0};
	double divider = dynamicScale(source, scaledSource, samples_read);

	applyWindow(scaledSource, hannWindow, samples_read);

	static int16_t ALIGN4 scratchData[512 * 2];

	// Split the data
	for(int i=0; i<samples_read*2; i+=2){
		scratchData[i] = scaledSource[i/2]; // Real
		scratchData[i+1] = 0; // Imaginary
	}

	arm_radix2_butterfly(scratchData, (int16_t)samples_read, (int16_t *)twiddleCoefQ15_512);
	arm_bitreversal(scratchData, samples_read, (uint16_t *)armBitRevTable8);

	for (int i=0; i<samples_read/2; i++)
	{
		// Calculate result and normalize SpectrumBuffer, also revert dynamic scaling
		uint32_t myReal = pow(scratchData[i*2], 2);
		uint32_t myImg = pow(scratchData[(i*2)+1], 2);

		readingFFT[i] = sqrt(myReal + myImg) * divider * 4;
	}

	// Exception for the first bin
	readingFFT[0] = readingFFT[0] / 2;

	return 0;
}
double dynamicScale(int32_t *source, int16_t *scaledSource, int samples_read)
{
	int32_t maxLevel = 0;
	for (uint16_t i=0; i<samples_read; i++) 
	{
		if (abs(source[i]) > maxLevel) 
		{
			maxLevel = abs(source[i]);
		}
	}
	double divider = (maxLevel+1) / 32768.0; // 16 bits
	if (divider < 1) divider = 1;

	for (uint16_t i=0; i<samples_read; i++) 
	{
		scaledSource[i] = source[i] / divider;
	}

	return divider;
}
void applyWindow(int16_t *src, const uint16_t *window, uint16_t len)
{ 
	/* This code is from https://github.com/adafruit/Adafruit_ZeroFFT thank you!
		-------
		This is an FFT library for ARM cortex M0+ CPUs
		Adafruit invests time and resources providing this open source code, 
		please support Adafruit and open-source hardware by purchasing products from Adafruit!
		Written by Dean Miller for Adafruit Industries. MIT license, all text above must be included in any redistribution
		------
	*/

	while(len--)
	{
		int32_t val = *src * *window++;
		*src = val >> 15;
		src++;
	}
}
void arm_radix2_butterfly(int16_t * pSrc, int16_t fftLen, int16_t * pCoef)
{
	/* This code is from https://github.com/adafruit/Adafruit_ZeroFFT thank you!
		-------
		This is an FFT library for ARM cortex M0+ CPUs
		Adafruit invests time and resources providing this open source code, 
		please support Adafruit and open-source hardware by purchasing products from Adafruit!
		Written by Dean Miller for Adafruit Industries. MIT license, all text above must be included in any redistribution
		------
	*/

	int i, j, k, l;
	int n1, n2, ia;
	int16_t xt, yt, cosVal, sinVal;

	n2 = fftLen;

	n1 = n2;
	n2 = n2 >> 1;
	ia = 0;

	// loop for groups
	for (j=0; j<n2; j++) 
	{
		cosVal = pCoef[ia * 2];
		sinVal = pCoef[(ia * 2) + 1];
		ia++;

		// loop for butterfly
		for (i=j; i<fftLen; i+=n1) 
		{
			l = i + n2;
			xt = (pSrc[2 * i] >> 2u) - (pSrc[2 * l] >> 2u);
			pSrc[2 * i] = ((pSrc[2 * i] >> 2u) + (pSrc[2 * l] >> 2u)) >> 1u;

			yt = (pSrc[2 * i + 1] >> 2u) - (pSrc[2 * l + 1] >> 2u);
			pSrc[2 * i + 1] =
				((pSrc[2 * l + 1] >> 2u) + (pSrc[2 * i + 1] >> 2u)) >> 1u;

			pSrc[2u * l] = (((int16_t) (((int32_t) xt * cosVal) >> 16)) +
					((int16_t) (((int32_t) yt * sinVal) >> 16)));

			pSrc[2u * l + 1u] = (((int16_t) (((int32_t) yt * cosVal) >> 16)) -
					((int16_t) (((int32_t) xt * sinVal) >> 16)));

		}                           // butterfly loop end
	}                             // groups loop end

	uint16_t twidCoefModifier = 2;

	// loop for stage
	for (k = fftLen / 2; k > 2; k = k >> 1) 
	{
		n1 = n2;
		n2 = n2 >> 1;
		ia = 0;

		// loop for groups
		for (j=0; j<n2; j++) 
		{
			cosVal = pCoef[ia * 2];
			sinVal = pCoef[(ia * 2) + 1];

			ia = ia + twidCoefModifier;

			// loop for butterfly
			for (i=j; i<fftLen; i+=n1) 
			{
				l = i + n2;
				xt = pSrc[2 * i] - pSrc[2 * l];
				pSrc[2 * i] = (pSrc[2 * i] + pSrc[2 * l]) >> 1u;

				yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
				pSrc[2 * i + 1] = (pSrc[2 * l + 1] + pSrc[2 * i + 1]) >> 1u;

				pSrc[2u * l] = (((int16_t) (((int32_t) xt * cosVal) >> 16)) +
						((int16_t) (((int32_t) yt * sinVal) >> 16)));

				pSrc[2u * l + 1u] = (((int16_t) (((int32_t) yt * cosVal) >> 16)) -
						((int16_t) (((int32_t) xt * sinVal) >> 16)));

			}                         // butterfly loop end
		}                           // groups loop end
		twidCoefModifier = twidCoefModifier << 1u;
	}                             // stages loop end

	n1 = n2;
	n2 = n2 >> 1;
	ia = 0;
	// loop for groups
	for (j=0; j<n2; j++) 
	{
		cosVal = pCoef[ia * 2];
		sinVal = pCoef[(ia * 2) + 1];

		ia = ia + twidCoefModifier;

		// loop for butterfly
		for (i=j; i<fftLen; i+=n1) 
		{
			l = i + n2;
			xt = pSrc[2 * i] - pSrc[2 * l];
			pSrc[2 * i] = (pSrc[2 * i] + pSrc[2 * l]);

			yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
			pSrc[2 * i + 1] = (pSrc[2 * l + 1] + pSrc[2 * i + 1]);

			pSrc[2u * l] = xt;

			pSrc[2u * l + 1u] = yt;

		}                           // butterfly loop end
	}                             // groups loop end
}
void arm_bitreversal(int16_t * pSrc16, uint32_t fftLen, uint16_t * pBitRevTab)
{
	/* This code is from https://github.com/adafruit/Adafruit_ZeroFFT thank you!
		-------
		This is an FFT library for ARM cortex M0+ CPUs
		Adafruit invests time and resources providing this open source code, 
		please support Adafruit and open-source hardware by purchasing products from Adafruit!
		Written by Dean Miller for Adafruit Industries. MIT license, all text above must be included in any redistribution
		------
	*/

	int32_t *pSrc = (int32_t *) pSrc16;
	int32_t in;
	uint32_t fftLenBy2, fftLenBy2p1;
	uint32_t i, j;

	/*  Initializations */
	j = 0u;
	fftLenBy2 = fftLen / 2u;
	fftLenBy2p1 = (fftLen / 2u) + 1u;

	/* Bit Reversal Implementation */
	for (i = 0u; i <= (fftLenBy2 - 2u); i += 2u) 
	{
		if(i < j) 
		{
			in = pSrc[i];
			pSrc[i] = pSrc[j];
			pSrc[j] = in;

			in = pSrc[i + fftLenBy2p1];
			pSrc[i + fftLenBy2p1] = pSrc[j + fftLenBy2p1];
			pSrc[j + fftLenBy2p1] = in;
		}

		in = pSrc[i + 1u];
		pSrc[i + 1u] = pSrc[j + fftLenBy2];
		pSrc[j + fftLenBy2] = in;

		/*  Reading the index for the bit reversal */
		j = *pBitRevTab;

		/*  Updating the bit reversal index depending on the fft length  */
		pBitRevTab++;
	}
}