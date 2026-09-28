import { Loading } from 'element-ui';

export default (text = 'Loading...') => {
  const loadingInstance  = Loading.service({
    lock: true,
    text: text,
    // spinner: 'el-icon-loading',
    background: 'rgba(0, 0, 0, 0.75)',
    customClass: 'custom-fullscreen-loading'
  });
  return loadingInstance
}
