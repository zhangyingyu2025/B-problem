"""Distinct discovery evidence for each of twenty unique channels."""
from source_track import SourceTrack


class ChannelManager:
    def __init__(self):
        self.tracks={};self.checked={c:set() for c in range(1,21)}
    def discovery_channels(self):
        if len(self.tracks)>=16: return []
        return [c for c in range(1,21) if c not in self.tracks and len(self.checked[c])<7]
    def discovery_complete(self): return not self.discovery_channels()
    def register(self,channel,site,response,position):
        if response['measure_result']=='no_signal':
            if site is not None and channel not in self.tracks: self.checked[channel].add(site)
            return None
        track=self.tracks.setdefault(channel,SourceTrack(channel))
        track.observe(position,response);return track
    def evidence(self):
        return {str(c):{'state':'DISCOVERED' if c in self.tracks else 'ABSENT_CERT' if len(self.checked[c])==7 else
                       'EXCLUDED_BY_COUNT_BOUND' if len(self.tracks)==16 else 'UNKNOWN',
                       'checked_sites':sorted(self.checked[c])} for c in range(1,21)}


def channel_order(channels,current,reverse=False):
    ordered=sorted(set(channels),reverse=reverse)
    if current in ordered: ordered.remove(current);ordered.insert(0,current)
    return ordered
