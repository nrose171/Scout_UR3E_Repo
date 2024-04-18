import pandas as pd

boat_df = pd.read_csv("/root/MM_scout_ur5e_proj/model/model_facets/boat.csv")
boat_df["x"] += 5
boat_df["y"] += 5
boat_df = boat_df.sort_values(['x', 'y', 'z'], ascending=[True, True, True])

first_wp = boat_df.iloc[0]