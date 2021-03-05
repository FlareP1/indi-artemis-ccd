/*
 Generic CCD
 CCD Template for INDI Developers
 Copyright (C) 2012 Jasem Mutlaq (mutlaqja@ikarustech.com)
 Multiple device support Copyright (C) 2013 Peter Polakovic (peter.polakovic@cloudmakers.eu)
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <memory>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <sys/time.h>

#include "config.h"
#include "indidevapi.h"
#include "eventloop.h"

#include "indi_artemis_ccd.h"
#include "ArtBase.h"

#define MAX_CCD_TEMP   45   /* Max CCD temperature */
#define MIN_CCD_TEMP   -55  /* Min CCD temperature */
#define MAX_X_BIN      16   /* Max Horizontal binning */
#define MAX_Y_BIN      16   /* Max Vertical binning */
#define MAX_PIXELS     4096 /* Max number of pixels in one dimension */
#define TEMP_THRESHOLD .25  /* Differential temperature threshold (C)*/
#define MAX_DEVICES    20   /* Max device cameraCount */

static int cameraCount;
static GenericCCD *cameras[MAX_DEVICES];

/**********************************************************
 *
 *  IMPORRANT: List supported camera models in initializer of deviceTypes structure
 *
 **********************************************************/

/*
static struct
{
    int vid;
    int pid;
    const char *name;
} deviceTypes[] = { { 0x0001, 0x0001, "Model 1" }, { 0x0001, 0x0002, "Model 2" }, { 0, 0, nullptr } };
*/

static void cleanup()
{
    for (int i = 0; i < cameraCount; i++)
    {
        delete cameras[i];
    }
}

void ISInit()
{
    static bool isInit = false;
    if (!isInit)
    {
        
        /**********************************************************
     *
     *  IMPORRANT: If available use CCD API function for enumeration available CCD's otherwise use code like this:
     *
     **********************************************************
     cameraCount = 0;
     for (struct usb_bus *bus = usb_get_busses(); bus && cameraCount < MAX_DEVICES; bus = bus->next) {
       for (struct usb_device *dev = bus->devices; dev && cameraCount < MAX_DEVICES; dev = dev->next) {
         int vid = dev->descriptor.idVendor;
         int pid = dev->descriptor.idProduct;
         for (int i = 0; deviceTypes[i].pid; i++) {
           if (vid == deviceTypes[i].vid && pid == deviceTypes[i].pid) {
             cameras[i] = new GenericCCD(dev, deviceTypes[i].name);
             break;
           }
         }
       }
     }
     */
    std::cout << "Starting Main!\n";

    ArtBase *m_ds = new ArtBase();

    std::cout << "ArtBase Constructed " << m_ds << " \n" ;

    //Serach for the Camera
    ArtBase::EnumerateArtCameras();
    // simply make an array of camera names to choose from
    // Do not clear contents here ! cameras.clear();

    std::cout << "ArtBase::EnumerateArtCameras() \n" ;

    wxArrayString* cameras_ptr = new wxArrayString();
    wxArrayString& cameraslist = *cameras_ptr;

    int num_items =  ArtBase::NumItems();
    if ( num_items >0 ) {
        for(int item=0; item<num_items; item++) {
            const ArtDevice& descr = ArtBase::ArtDevEntry(item);
            // add camera friendly name to list
            std::cout << "Found Camera " << descr.DevName() << "\n";

            cameras[cameraCount] = new GenericCCD(m_ds, cameraCount, descr.DevName());
            cameraslist.Add(descr.DevName()); 
            cameraCount++;        
        }
    }

        /* For demo purposes we are creating two test devices */
        //cameraCount            = 2;
        //struct usb_device *dev = nullptr;
        //cameras[0]             = new GenericCCD(dev, deviceTypes[0].name);
        //cameras[1]             = new GenericCCD(dev, deviceTypes[1].name);

        atexit(cleanup);
        isInit = true;
        //LOG_INFO("Artemis Init Complete.");
    }
}

void ISGetProperties(const char *dev)
{
    ISInit();
    for (int i = 0; i < cameraCount; i++)
    {
        GenericCCD *camera = cameras[i];
        if (dev == nullptr || !strcmp(dev, camera->name))
        {
            camera->ISGetProperties(dev);
            if (dev != nullptr)
                break;
        }
    }
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int num)
{
    ISInit();
    for (int i = 0; i < cameraCount; i++)
    {
        GenericCCD *camera = cameras[i];
        if (dev == nullptr || !strcmp(dev, camera->name))
        {
            camera->ISNewSwitch(dev, name, states, names, num);
            if (dev != nullptr)
                break;
        }
    }
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int num)
{
    ISInit();
    for (int i = 0; i < cameraCount; i++)
    {
        GenericCCD *camera = cameras[i];
        if (dev == nullptr || !strcmp(dev, camera->name))
        {
            camera->ISNewText(dev, name, texts, names, num);
            if (dev != nullptr)
                break;
        }
    }
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int num)
{
    ISInit();
    for (int i = 0; i < cameraCount; i++)
    {
        GenericCCD *camera = cameras[i];
        if (dev == nullptr || !strcmp(dev, camera->name))
        {
            camera->ISNewNumber(dev, name, values, names, num);
            if (dev != nullptr)
                break;
        }
    }
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[],
               char *names[], int n)
{
    INDI_UNUSED(dev);
    INDI_UNUSED(name);
    INDI_UNUSED(sizes);
    INDI_UNUSED(blobsizes);
    INDI_UNUSED(blobs);
    INDI_UNUSED(formats);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
}
void ISSnoopDevice(XMLEle *root)
{
    ISInit();

    for (int i = 0; i < cameraCount; i++)
    {
        GenericCCD *camera = cameras[i];
        camera->ISSnoopDevice(root);
    }
}

GenericCCD::GenericCCD(ArtBase *m_ds, int deviceIndex, const char *name)
{
    LOG_INFO("Artemis CCD Constructor.");

    this->m_ds = m_ds;
    this->deviceIndex = deviceIndex;

    snprintf(this->name, 32, "Generic CCD %s", name);
    setDeviceName(this->name);

    setVersion(CDRIVER_VERSION_MAJOR, CDRIVER_VERSION_MINOR);

    LOG_INFO("Artemis CCD Constructor complete.");
}

GenericCCD::~GenericCCD()
{
    if(m_ds!=nullptr)
    {
        m_ds->DropControlledDevice(); // drop cam object
        sleep(1);
        delete(m_ds);
    }
}

const char *GenericCCD::getDefaultName()
{
    return "Generic CCD";
}

bool GenericCCD::initProperties()
{
    // Init parent properties first
    //LOG_INFO("Artemis CCD initProperties.");

    INDI::CCD::initProperties();

    //LOG_INFO("Artemis CCD initProperties 1.");

    //uint32_t cap = CCD_CAN_ABORT | CCD_CAN_BIN | CCD_CAN_SUBFRAME | CCD_HAS_COOLER | CCD_HAS_SHUTTER | CCD_HAS_ST4_PORT;
    uint32_t cap = CCD_CAN_ABORT | CCD_CAN_BIN | CCD_CAN_SUBFRAME;
    SetCCDCapability(cap);

    //LOG_INFO("Artemis CCD initProperties 2.");

    addConfigurationControl();
    addDebugControl();

    //LOG_INFO("Artemis CCD initProperties Complete.");
    return true;
}

void GenericCCD::ISGetProperties(const char *dev)
{
    INDI::CCD::ISGetProperties(dev);
}

bool GenericCCD::updateProperties()
{
    INDI::CCD::updateProperties();

    if (isConnected())
    {
        // Let's get parameters now from CCD
        setupParams();

        timerID = SetTimer(getPollingPeriod());
    }
    else
    {
        rmTimer(timerID);
    }

    return true;
}

bool GenericCCD::Connect()
{
    LOG_INFO("Attempting to find the Generic CCD...");

    /**********************************************************
   *
   *
   *
   *  IMPORRANT: Put here your CCD Connect function
   *  If you encameraCounter an error, send the client a message
   *  e.g.
   *  LOG_INFO( "Error, unable to connect due to ...");
   *  return false;
   *
   *
   **********************************************************/
    bool status = m_ds->ControlDevice(deviceIndex);
   
    /* Success! */
    LOG_INFO("CCD is online. Retrieving basic data.");

    return status;
}

bool GenericCCD::Disconnect()
{
    /**********************************************************
   *
   *
   *
   *  IMPORRANT: Put here your CCD disonnect function
   *  If you encameraCounter an error, send the client a message
   *  e.g.
   *  LOG_INFO( "Error, unable to disconnect due to ...");
   *  return false;
   *
   *
   **********************************************************/
    m_ds->DropControlledDevice();

    LOG_INFO("CCD is offline.");
    return true;
}

bool GenericCCD::setupParams()
{
    float x_pixel_size, y_pixel_size;
    int bit_depth = 16;
    int x_1, y_1, x_2, y_2;


    LOG_INFO("setupParams.");

    /**********************************************************
   *
   *
   *
   *  IMPORRANT: Get basic CCD parameters here such as
   *  + Pixel Size X
   *  + Pixel Size Y
   *  + Bit Depth?
   *  + X, Y, W, H of frame
   *  + Temperature
   *  + ...etc
   *
   *
   *
   **********************************************************/

    ///////////////////////////
    // 1. Get Pixel size
    ///////////////////////////
    // Actucal CALL to CCD to get pixel size here
    x_pixel_size = 5.4;
    y_pixel_size = 5.4;
    this->m_ds->CcdPixelSizeDimension(x_pixel_size, y_pixel_size);

    ///////////////////////////
    // 2. Get Frame
    ///////////////////////////

    // Actucal CALL to CCD to get frame information here
    x_1 = y_1 = 0;
    x_2       = 1280;
    y_2       = 1024;
    unsigned short ccdx , ccdy;
    this->m_ds->CcdDimension(ccdx, ccdy);
    x_2 = int(ccdx);
    y_2 = int(ccdy);

    ///////////////////////////
    // 3. Get temperature
    ///////////////////////////
    // Setting sample temperature -- MAKE CALL TO API FUNCTION TO GET TEMPERATURE IN REAL DRIVER
    TemperatureN[0].value = 25.0;
    LOGF_INFO("The CCD Temperature is %f", TemperatureN[0].value);
    IDSetNumber(&TemperatureNP, nullptr);

    ///////////////////////////
    // 4. Get temperature
    ///////////////////////////
    bit_depth = 16;
    SetCCDParams(x_2 - x_1, y_2 - y_1, bit_depth, x_pixel_size, y_pixel_size);

    // Now we usually do the following in the hardware
    // Set Frame to LIGHT or NORMAL
    // Set Binning to 1x1
    /* Default frame type is NORMAL */

    // Let's calculate required buffer
    int nbuf;
    nbuf = PrimaryCCD.getXRes() * PrimaryCCD.getYRes() * PrimaryCCD.getBPP() / 8; //  this is pixel cameraCount
    nbuf += 512;                                                                  //  leave a little extra at the end
    PrimaryCCD.setFrameBufferSize(nbuf);

    return true;
}

int GenericCCD::SetTemperature(double temperature)
{
    // If there difference, for example, is less than 0.1 degrees, let's immediately return OK.
    if (fabs(temperature - TemperatureN[0].value) < TEMP_THRESHOLD)
        return 1;

    /**********************************************************
     *
     *  IMPORRANT: Put here your CCD Set Temperature Function
     *  We return 0 if setting the temperature will take some time
     *  If the requested is the same as current temperature, or very
     *  close, we return 1 and INDI::CCD will mark the temperature status as OK
     *  If we return 0, INDI::CCD will mark the temperature status as BUSY
     **********************************************************/

    // Otherwise, we set the temperature request and we update the status in TimerHit() function.
    TemperatureRequest = temperature;
    LOGF_INFO("Setting CCD temperature to %+06.2f C", temperature);
    return 0;
}

bool GenericCCD::StartExposure(float duration)
{
    if (duration < minDuration)
    {
        DEBUGF(INDI::Logger::DBG_WARNING,
               "Exposure shorter than minimum duration %g s requested. \n Setting exposure time to %g s.", duration,
               minDuration);
        duration = minDuration;
    }

    if (imageFrameType == INDI::CCDChip::BIAS_FRAME)
    {
        duration = minDuration;
        LOGF_INFO("Bias Frame (s) : %g\n", minDuration);
    }

    /**********************************************************
   *
   *
   *
   *  IMPORRANT: Put here your CCD start exposure here
   *  Please note that duration passed is in seconds.
   *  If there is an error, report it back to client
   *  e.g.
   *  LOG_INFO( "Error, unable to start exposure due to ...");
   *  return -1;
   *
   *
   **********************************************************/
    unsigned long capture_duration_ms = (unsigned long)(duration * 1000);
    m_ds->CaptureImage(true, capture_duration_ms);

    PrimaryCCD.setExposureDuration(duration);
    ExposureRequest = duration;

    gettimeofday(&ExpStart, nullptr);
    LOGF_INFO("Taking a %g seconds frame...", ExposureRequest);

    InExposure = true;

    return true;
}

bool GenericCCD::AbortExposure()
{
    /**********************************************************
   *
   *
   *
   *  IMPORRANT: Put here your CCD abort exposure here
   *  If there is an error, report it back to client
   *  e.g.
   *  LOG_INFO( "Error, unable to abort exposure due to ...");
   *  return false;
   *
   *
   **********************************************************/

    m_ds->AbortExposure();

    InExposure = false;
    return true;
}

bool GenericCCD::UpdateCCDFrameType(INDI::CCDChip::CCD_FRAME fType)
{
    INDI::CCDChip::CCD_FRAME imageFrameType = PrimaryCCD.getFrameType();

    if (fType == imageFrameType)
        return true;

    switch (imageFrameType)
    {
        case INDI::CCDChip::BIAS_FRAME:
        case INDI::CCDChip::DARK_FRAME:
            /**********************************************************
     *
     *
     *
     *  IMPORRANT: Put here your CCD Frame type here
     *  BIAS and DARK are taken with shutter closed, so _usually_
     *  most CCD this is a call to let the CCD know next exposure shutter
     *  must be closed. Customize as appropiate for the hardware
     *  If there is an error, report it back to client
     *  e.g.
     *  LOG_INFO( "Error, unable to set frame type to ...");
     *  return false;
     *
     *
     **********************************************************/
            break;

        case INDI::CCDChip::LIGHT_FRAME:
        case INDI::CCDChip::FLAT_FRAME:
            /**********************************************************
     *
     *
     *
     *  IMPORRANT: Put here your CCD Frame type here
     *  LIGHT and FLAT are taken with shutter open, so _usually_
     *  most CCD this is a call to let the CCD know next exposure shutter
     *  must be open. Customize as appropiate for the hardware
     *  If there is an error, report it back to client
     *  e.g.
     *  LOG_INFO( "Error, unable to set frame type to ...");
     *  return false;
     *
     *
     **********************************************************/
            break;
    }

    PrimaryCCD.setFrameType(fType);

    return true;
}

bool GenericCCD::UpdateCCDFrame(int x, int y, int w, int h)
{
    /* Add the X and Y offsets */
    long x_1 = x;
    long y_1 = y;

    long bin_width  = x_1 + (w / PrimaryCCD.getBinX());
    long bin_height = y_1 + (h / PrimaryCCD.getBinY());

    if (bin_width > PrimaryCCD.getXRes() / PrimaryCCD.getBinX())
    {
        LOGF_INFO("Error: invalid width requested %d", w);
        return false;
    }
    else if (bin_height > PrimaryCCD.getYRes() / PrimaryCCD.getBinY())
    {
        LOGF_INFO("Error: invalid height request %d", h);
        return false;
    }

    /**********************************************************
   *
   *
   *
   *  IMPORRANT: Put here your CCD Frame dimension call
   *  The values calculated above are BINNED width and height
   *  which is what most CCD APIs require, but in case your
   *  CCD API implementation is different, don't forget to change
   *  the above calculations.
   *  If there is an error, report it back to client
   *  e.g.
   *  LOG_INFO( "Error, unable to set frame to ...");
   *  return false;
   *
   *
   **********************************************************/

    // Set UNBINNED coords
    PrimaryCCD.setFrame(x_1, y_1, w, h);

    int nbuf;
    nbuf = (bin_width * bin_height * PrimaryCCD.getBPP() / 8); //  this is pixel count
    nbuf += 512;                                               //  leave a little extra at the end
    PrimaryCCD.setFrameBufferSize(nbuf);

    LOGF_DEBUG("Setting frame buffer size to %d bytes.", nbuf);

    return true;
}

bool GenericCCD::UpdateCCDBin(int binx, int biny)
{
    /**********************************************************
   *
   *
   *
   *  IMPORRANT: Put here your CCD Binning call
   *  If there is an error, report it back to client
   *  e.g.
   *  LOG_INFO( "Error, unable to set binning to ...");
   *  return false;
   *
   *
   **********************************************************/
    /* Artemis camera only support symetrical binning */
    binx = max(binx,biny);
    biny = binx;
    m_ds->SetFormat(binx);

    PrimaryCCD.setBin(binx, biny);

    return UpdateCCDFrame(PrimaryCCD.getSubX(), PrimaryCCD.getSubY(), PrimaryCCD.getSubW(), PrimaryCCD.getSubH());
}

float GenericCCD::CalcTimeLeft()
{
    double timesince;
    double timeleft;
    struct timeval now;
    bool exposing;
    float timeRemaining;

    gettimeofday(&now, nullptr);

    timesince = (double)(now.tv_sec * 1000.0 + now.tv_usec / 1000) -
                (double)(ExpStart.tv_sec * 1000.0 + ExpStart.tv_usec / 1000);
    timesince = timesince / 1000;

    timeleft = ExposureRequest - timesince;

    //This will report the actual time remaining includeing the loading of the image
    //but intially it will just show exposing at m_timeRemaining is 0
    //then m_timeRemaining gets set to 100 and decreaeses to 0?
    m_ds->TimeRemaining(exposing , timeRemaining);

    return max(timeleft,(double)timeRemaining);
}

/* Downloads the image from the CCD.
 N.B. No processing is done on the image */
int GenericCCD::grabImage()
{
    uint8_t *image = PrimaryCCD.getFrameBuffer();
    int width      = PrimaryCCD.getSubW() / PrimaryCCD.getBinX() * PrimaryCCD.getBPP() / 8;
    int height     = PrimaryCCD.getSubH() / PrimaryCCD.getBinY();

    DEBUGF(INDI::Logger::DBG_SESSION, "Internal Image Size w %i h %i\n",width, height);

     /**********************************************************
     *
     *
     *  IMPORRANT: Put here your CCD Get Image routine here
     *  use the image, width, and height variables above
     *  If there is an error, report it back to client
     *
     *
     **********************************************************/
    
    wxArtSample *m_pArtSample = new wxArtSample();

    // allow the camcode to get the samples
    long capResult = m_ds->OnCapture(); // allow the camcode to get the samples

    if(capResult == 0  )
    {
        std::cout << "CapturedSample " <<  capResult << " \n";
        m_ds->CapturedSample(*m_pArtSample);  // we _must_ collect the sample if it is ready

    }
    
    //*extract the image from 16bit frame to sudo 8 bit fame
    // Width is set larger to simulate 16bits
    uint8_t *cameraImage_ptr = (uint8_t *)m_pArtSample->SampleYPtr();
    int hSize = m_pArtSample->SampleSize().GetHeight();
    int wSize = m_pArtSample->SampleSize().GetWidth() * PrimaryCCD.getBPP()/8;

    DEBUGF(INDI::Logger::DBG_SESSION, "Downloaded Image Size w %i h %i\n",wSize ,hSize);

    #if 1
    for (int i = 0; i < hSize ; i++)
    {
        for (int j = 0; j < wSize; j++)
        {
            //image[i * width + j] = rand() % 255;
            image[i * wSize + j] = cameraImage_ptr[i * wSize + j];
        }
    }
    #endif

    //Free the memory from the download.
    delete m_pArtSample;

#if 0
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            image[i * width + j] = rand() % 255;
        }
    }
#endif



    LOG_INFO("Download complete.");

    ExposureComplete(&PrimaryCCD);

    return 0;
}

void GenericCCD::TimerHit()
{
    int timerID = -1;
    long timeleft;

    LOG_INFO("TimerHit.");

    if (isConnected() == false)
        return; //  No need to reset timer if we are not connected anymore

    if (InExposure)
    {
        timeleft = CalcTimeLeft();

        if (timeleft < 1.0)
        {
            if (timeleft > 0.25)
            {
                //  a quarter of a second or more
                //  just set a tighter timer
                timerID = SetTimer(250);
            }
            else
            {
                if (timeleft > 0.07)
                {
                    //  use an even tighter timer
                    timerID = SetTimer(50);
                }
                else
                {
                    ESYNMode status = m_ds->OnCaptureRequired();
                    //  it's real close now, so spin on it
                    while (status != ESYNMode::ESYN_All )  //Wait until capture and download complete
                    {
                        /**********************************************************
             *
             *  IMPORRANT: If supported by your CCD API
             *  Add a call here to check if the image is ready for download
             *  If image is ready, set timeleft to 0. Some CCDs (check FLI)
             *  also return timeleft in msec.
             *
             **********************************************************/
                        
                        status = m_ds->OnCaptureRequired();
                        usleep(1000000 * 0.1f);

                        // Breaking in simulation, in real driver either loop until time left = 0 or use an API call to know if the image is ready for download
                        break;

                        //int slv;
                        //slv = 100000 * timeleft;
                        //usleep(slv);
                    }

                    /* We're done exposing */
                    LOG_INFO("Exposure done, downloading image...");

                    long capResult;
                    do
                    {
                        // allow the camcode to get the samples
                        capResult = m_ds->OnCapture(); // allow the camcode to get the samples
                        std::cout << "capResult " << capResult << "\n";
                        sleep(1);
                    } while (capResult != 0);
                   
                    PrimaryCCD.setExposureLeft(0);
                    InExposure = false;
                    /* grab and save image */
                    grabImage();
                }
            }
        }
        else
        {
            if (isDebug())
            {
                IDLog("With time left %ld\n", timeleft);
                IDLog("image not yet ready....\n");
            }

            PrimaryCCD.setExposureLeft(timeleft);
        }
    }

    switch (TemperatureNP.s)
    {
        case IPS_IDLE:
        case IPS_OK:
            /**********************************************************
     *
     *
     *
     *  IMPORRANT: Put here your CCD Get temperature call here
     *  If there is an error, report it back to client
     *  e.g.
     *  LOG_INFO( "Error, unable to get temp due to ...");
     *  return false;
     *
     *
     **********************************************************/
            break;

        case IPS_BUSY:
            /**********************************************************
       *
       *
       *
       *  IMPORRANT: Put here your CCD Get temperature call here
       *  If there is an error, report it back to client
       *  e.g.
       *  LOG_INFO( "Error, unable to get temp due to ...");
       *  return false;
       *
       *
       **********************************************************/
            TemperatureN[0].value = TemperatureRequest;

            // If we're within threshold, let's make it BUSY ---> OK
            if (fabs(TemperatureRequest - TemperatureN[0].value) <= TEMP_THRESHOLD)
                TemperatureNP.s = IPS_OK;

            IDSetNumber(&TemperatureNP, nullptr);
            break;

        case IPS_ALERT:
            break;
    }

    if (timerID == -1)
        SetTimer(getPollingPeriod());
    return;
}

IPState GenericCCD::GuideNorth(uint32_t ms)
{
    INDI_UNUSED(ms);
    /**********************************************************
   *
   *
   *
   *  IMPORRANT: Put here your CCD Guide call
   *  Some CCD API support pulse guiding directly (i.e. without timers)
   *  Others implement GUIDE_ON and GUIDE_OFF for each direction, and you
   *  will have to start a timer and then stop it after the 'ms' milliseconds
   *  For an example on timer usage, please refer to indi-sx and indi-gpusb drivers
   *  available in INDI 3rd party repository
   *  If there is an error, report it back to client
   *  e.g.
   *  LOG_INFO( "Error, unable to guide due ...");
   *  return IPS_ALERT;
   *
   *
   **********************************************************/

    return IPS_OK;
}

IPState GenericCCD::GuideSouth(uint32_t ms)
{
    INDI_UNUSED(ms);
    /**********************************************************
     *
     *
     *
     *  IMPORRANT: Put here your CCD Guide call
     *  Some CCD API support pulse guiding directly (i.e. without timers)
     *  Others implement GUIDE_ON and GUIDE_OFF for each direction, and you
     *  will have to start a timer and then stop it after the 'ms' milliseconds
     *  For an example on timer usage, please refer to indi-sx and indi-gpusb drivers
     *  available in INDI 3rd party repository
     *  If there is an error, report it back to client
     *  e.g.
     *  LOG_INFO( "Error, unable to guide due ...");
     *  return IPS_ALERT;
     *
     *
     **********************************************************/

    return IPS_OK;
}

IPState GenericCCD::GuideEast(uint32_t ms)
{
    INDI_UNUSED(ms);
    /**********************************************************
     *
     *
     *
     *  IMPORRANT: Put here your CCD Guide call
     *  Some CCD API support pulse guiding directly (i.e. without timers)
     *  Others implement GUIDE_ON and GUIDE_OFF for each direction, and you
     *  will have to start a timer and then stop it after the 'ms' milliseconds
     *  For an example on timer usage, please refer to indi-sx and indi-gpusb drivers
     *  available in INDI 3rd party repository
     *  If there is an error, report it back to client
     *  e.g.
     *  LOG_INFO( "Error, unable to guide due ...");
     *  return IPS_ALERT;
     *
     *
     **********************************************************/

    return IPS_OK;
}

IPState GenericCCD::GuideWest(uint32_t ms)
{
    INDI_UNUSED(ms);
    /**********************************************************
     *
     *
     *
     *  IMPORRANT: Put here your CCD Guide call
     *  Some CCD API support pulse guiding directly (i.e. without timers)
     *  Others implement GUIDE_ON and GUIDE_OFF for each direction, and you
     *  will have to start a timer and then stop it after the 'ms' milliseconds
     *  For an example on timer usage, please refer to indi-sx and indi-gpusb drivers
     *  available in INDI 3rd party repository
     *  If there is an error, report it back to client
     *  e.g.
     *  LOG_INFO( "Error, unable to guide due ...");
     *  return IPS_ALERT;
     *
     *
     **********************************************************/

    return IPS_OK;
}

