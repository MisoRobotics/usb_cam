import asyncio
import os
import onvif
from onvif import ONVIFCamera

async def main():
    wsdl_path = os.path.join(os.path.dirname(onvif.__file__), "wsdl")
    
    print("Connecting to camera at 192.168.41.63...")
    mycam = ONVIFCamera('192.168.41.63', 80, 'admin', 'camera', wsdl_dir=wsdl_path)
    
    try:
        await mycam.update_xaddrs()
        media_service = await mycam.create_media_service()
        
        # Get the encoders
        encoders = await media_service.GetVideoEncoderConfigurations()
        if not encoders:
            print("No video encoder configurations found.")
            return
            
        target_encoder = encoders[0]
        print(f"Modifying Encoder Token: '{target_encoder.token}'")

        # --- MATCHING YOUR IMAGE SETTINGS (STRICT SCHEMA MODE) ---
        
        # 1. Encoding Codec (H.264)
        target_encoder.Encoding = 'H264'

        # 2. Resolution (4K UHD: 3840 x 2160)
        target_encoder.Resolution.Width = 3840
        target_encoder.Resolution.Height = 2160

        # 3. Frame Rate (20 fps)
        target_encoder.RateControl.FrameRateLimit = 20

        # 4. Maximum Bit Rate (8000 kbps)
        target_encoder.RateControl.BitrateLimit = 8000

        # 5. GOP Size & Profile (Routed into the H264 sub-configuration block)
        if hasattr(target_encoder, 'H264') and target_encoder.H264 is not None:
            target_encoder.H264.GovLength = 20
            target_encoder.H264.H264Profile = 'High'
            
        # 6. Delete top-level properties that caused previous schema validation errors
        if hasattr(target_encoder, 'GovLength'):
            delattr(target_encoder, 'GovLength')

        # --- PUSH CHANGES TO CAMERA ---
        print("\nApplying settings to match the image layout...")
        await media_service.SetVideoEncoderConfiguration({
            'Configuration': target_encoder, 
            'ForcePersistence': True
        })
        print("Success! Settings applied and saved to persistent memory.")
        
    except Exception as e:
        print(f"An error occurred: {e}")
        
    finally:
        print("Closing connection safely...")
        await mycam.close()

if __name__ == '__main__':
    asyncio.run(main())
