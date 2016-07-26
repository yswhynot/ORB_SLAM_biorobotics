'''
 Pseudo code for ORB_SLAM2
 Author: Sha Yi
'''

<System>
	System() {

		mpVocabulary = new ORBVocabulary();

		mpKeyFrameDatabase = new KeyFrameDatabase(mpVocabulary);
		mpMap = new Map();

		tracker_thread
		mapper_thread.run
		viewer_thread.run
	}

	TrackMonocular() {
		<Tracking>
			GrabImageMonocular() {
				if(!initialized) {
					<ORBExtractor> mpIniORBextractor = new ORBextractor(); </ORBExtractor>
				}
				Track();
			}

			Track() {
				if(!initialized) {
					MonocularInitialization();
				} else {
					
				}
			}

			MonocularInitialization() {
				mCurrentFrame = Frame(.., mpIniORBextractor, ..);
				<Frame>
				Frame() {
					ExtractORB();
					UndistortKeyPoints();
					if(initial)
						ComputeImageBounds();
					AssignFeaturesToGrid() {
						FRAME_GRID_ROWS = 48
						FRAME_GRID_COLS = 64
					}
				}
				</Frame>
				<ORBMatcher> ORBmatcher matcher(nnratio = 0.9, checkOrientation = true); </ORBMatcher>

				if(matcher_num < 100)
					return

				Initializer();
				CreateInitialMapMonocular() {
					update_all
					<Optimizer> GlobalBundleAdjustemnt(iteration = 20); </Optimizer>
				}
			}
		</Tracking>
	}
</System>

<ORBExtractor>
	ORBextractor() {
		resize_everything
		factor = 1.0f / scaleFactor;
		nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - pow(factor, nlevels));
		do_ORB_settings
	}
</ORBExtractor>

<ORBMatcher>
	SearchForInitialization() {
		TH_LOW = 50;
		HISTO_LENGTH = 30;
		<Frame>
			frame.GetFeaturesInArea() {
				knn_top_2
				match_orientation
			}
		</Frame>
	}
</ORBMatcher>

<Initializer>
	Initialize() {
		score_h = FindHomography();
		score_f = FindFundamental();
		score_r = score_h/(score_h + score_f);
		if(score_r > 0.4)
			return ReconstructH();
		else 
			return ReconstructF();
	}
</Initializer>