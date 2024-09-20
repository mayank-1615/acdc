#### Structure of the Bagfiles_orignal directory 

1. EUKF_bagfiles: Pre-precosses bag file after noise addition to ground truth for EKF and UKF implementation in CTRA motion model. 
2. KF_bagfiles: bag files after noise addition to ground truth for KF in CA motion model. 
3. Recordings
	EKF: recorded bagfile for fused object for EKF (further used for evaluation)
	KF: recorded bagfile for fused object for EKF (further used for evaluation)
	UKF: recorded bagfile for fused object for EKF (further used for evaluation)

Bagfiles_original
├── EUKF_bagfiles
│   ├── acdc_fusion_guidance_cauchy_noise.bag
│   ├── acdc_fusion_guidance_laplace_noise.bag
│   ├── acdc_fusion_guidance_lognorm_variatenoise.bag
│   └── acdc_fusion_guidance_gaussian_noise.bag
├── KF_bagfiles
│   ├── acdc_fusion_guidance_cauchy_noise.bag
│   ├── acdc_fusion_guidance_laplace_noise.bag
│   ├── acdc_fusion_guidance_lognormvariate_noise.bag
│   └── acdc_fusion_guidance_gaussian_noise.bag
└── Recordings
    ├── EKF
    │   ├── recording_cauchy_ekf.bag 
    │   ├── recording_gaussian_ekf.bag
    │   ├── recording_laplace_ekf.bag
    │   └── recording_lognormvariate_ekf.bag
    ├── KF
    │   ├── recording_cauchy_kf.bag
    │   ├── recording_gaussian_kf.bag
    │   ├── recording_laplace_kf.bag
    │   └── recording_lognormvariate_kf.bag
    └── UKF
        ├── recording_cauchy_ukf.bag
        ├── recording_gaussian_ukf.bag
        ├── recording_laplace_ukf.bag
        └── recording_lognormvariate_ukf.bag


