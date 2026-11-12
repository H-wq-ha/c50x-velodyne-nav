"""Temporary VLP16 quiet startup; restore the original settings on every exit."""
import json
import time
import urllib.parse
import urllib.request


class QuietMotorStartup:
    def __init__(self, host='192.168.1.201', clock=time.monotonic):
        self.base='http://'+host
        self.clock=clock
        self.original=None
        self.changed=False
        self.resume_started=None
        self.stable_since=None
        self.events=[]

    def _get(self,path):
        with urllib.request.urlopen(self.base+path,timeout=3) as response:
            return json.loads(response.read())

    def _set(self,values):
        request=urllib.request.Request(self.base+'/cgi/setting',
            data=urllib.parse.urlencode(values).encode(),method='POST')
        with urllib.request.urlopen(request,timeout=3) as response:
            if response.status not in (200,204):raise RuntimeError('雷达设置失败')

    def pause(self):
        self.original=self._get('/cgi/settings.json')
        if not isinstance(self.original.get('rpm'),int) or self.original.get('laser') not in ('On','Off'):
            raise RuntimeError('无法读取雷达原始转速与激光状态')
        self.changed=True
        self._set({'rpm':0})
        end=self.clock()+12
        while self.clock()<end:
            status=self._get('/cgi/status.json')
            if status['motor']['rpm']<5 and status['laser']['state']!='On':
                self.events.append(dict(stage='quiet_before_imu_bias',status=status))
                return
            time.sleep(.2)
        raise RuntimeError('雷达未及时停转，未开始静止零偏检查')

    def resume(self):
        self._set({'rpm':600})
        self._set({'laser':'on'})
        self.resume_started=self.clock()
        self.stable_since=None

    def ready(self):
        status=self._get('/cgi/status.json')
        now=self.clock()
        if abs(status['motor']['rpm']-600)<=6 and status['laser']['state']=='On':
            if self.stable_since is None:self.stable_since=now
            if now-self.stable_since>=1:
                self.events.append(dict(stage='capture_rpm_ready',status=status))
                return True
        else:self.stable_since=None
        if now-self.resume_started>15:raise RuntimeError('雷达未稳定恢复到 600 转/分钟')
        return False

    def restore(self):
        if not self.changed:return
        self._set({'rpm':self.original['rpm']})
        self._set({'laser':self.original['laser'].lower()})
        settings=self._get('/cgi/settings.json')
        if settings['rpm']!=self.original['rpm'] or settings['laser']!=self.original['laser']:
            raise RuntimeError('雷达原始转速/激光设置尚未恢复，请检查雷达状态')
        self.events.append(dict(stage='original_settings_restored',rpm=settings['rpm'],laser=settings['laser']))
        self.changed=False
