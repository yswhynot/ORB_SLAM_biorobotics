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
					mpIniORBextractor = new ORBextractor();
					<ORBExtractor>
						ORBextractor() {
							resize_everything
							factor = 1.0f / scaleFactor;
							nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - pow(factor, nlevels));
							do_ORB_settings
						}
					</ORBExtractor>
				}
				Track();
			}

			Track() {
				if(!initialized) {
					MonocularInitialization();
				} else {
					ORBmatcher matcher(nnratio = 0.9, checkOrientation = true);
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
			}
		</Tracking>
	}
</System>